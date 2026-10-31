//! Wi-Fi station and the embassy-net TCP/IP stack.
//!
//! Credentials come from `.cargo/secrets.toml` (see `secrets.toml.example`)
//! and are baked in at compile time.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{AuthenticationMethodConfig, Config, Interface, WifiController};
use log::{info, warn};
use static_cell::StaticCell;

use crate::acquisition::UNACKED;
use crate::stream::CONNECTED;

const SSID: &str = env!(
    "WIFI_SSID",
    "set WIFI_SSID in firmware/.cargo/secrets.toml, see secrets.toml.example"
);
const PASSWORD: &str = env!(
    "WIFI_PASSWORD",
    "set WIFI_PASSWORD in firmware/.cargo/secrets.toml, see secrets.toml.example"
);

const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const MONITOR_PERIOD: Duration = Duration::from_secs(1);
/// Seconds of a growing backlog before forcing a reconnect.
const STALL_LIMIT: u32 = 5;
/// Unacknowledged records that count as falling behind: 1 s at 2 kSPS.
const STALL_BACKLOG: u32 = 2000;
/// Stalls are not counted for this long after the IP comes up.
const STALL_GRACE: Duration = Duration::from_secs(10);

/// Signal strength of the access point in dBm; 0 while not connected.
pub static RSSI_DBM: AtomicI32 = AtomicI32::new(0);
/// Times the link dropped after a successful connection.
pub static DISCONNECTS: AtomicU32 = AtomicU32::new(0);
/// Reconnects forced because the link was too slow, see [`monitor_link`].
pub static STALL_RECONNECTS: AtomicU32 = AtomicU32::new(0);

/// DHCP client, the data TCP socket and the discovery UDP socket.
const SOCKETS: usize = 3;
static RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();

/// Starts Wi-Fi and the network stack in the background.
///
/// The returned stack is usable right away; sockets work once
/// [`Stack::wait_config_up`] returns.
pub fn start(spawner: Spawner, wifi: WIFI<'static>) -> Stack<'static> {
    let controller = WifiController::new(wifi, Default::default()).expect("Wi-Fi init");

    let (stack, runner) = embassy_net::new(
        Interface::station(),
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        random_seed(),
    );

    spawner.spawn(connection_task(controller, stack).expect("task pool is free"));
    spawner.spawn(net_task(runner).expect("task pool is free"));
    spawner.spawn(link_monitor_task(stack).expect("task pool is free"));
    stack
}

/// Seed for TCP sequence numbers, DHCP transaction ids, ephemeral ports.
fn random_seed() -> u64 {
    // The hardware RNG is a true RNG while the radio is enabled.
    let rng = Rng::new();
    u64::from(rng.random()) << 32 | u64::from(rng.random())
}

/// Connects to the access point and reconnects whenever the link drops.
#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>, stack: Stack<'static>) {
    let config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().expect("valid SSID"))
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().expect("valid password"),
            )),
    );
    controller.set_config(&config).expect("valid Wi-Fi config");

    loop {
        match controller.connect_async().await {
            Ok(_) => {
                info!("Wi-Fi connected to {SSID:?}");
                // Both take `&self`, so the link can be monitored while
                // waiting for the disconnect.
                let disconnect = controller.wait_for_disconnect_async();
                let monitor = monitor_link(&controller, stack);
                match select(disconnect, monitor).await {
                    Either::First(reason) => {
                        DISCONNECTS.fetch_add(1, Ordering::Relaxed);
                        warn!("Wi-Fi disconnected: {reason:?}");
                    }
                    Either::Second(()) => {
                        // The borrows of `select` are over: `&mut` is fine.
                        STALL_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                        warn!("Wi-Fi too slow for {STALL_LIMIT} s, reconnecting");
                        if let Err(e) = controller.disconnect_async().await {
                            warn!("Wi-Fi disconnect failed: {e:?}");
                        }
                    }
                }
                RSSI_DBM.store(0, Ordering::Relaxed);
            }
            Err(e) => warn!("Wi-Fi connect failed: {e:?}"),
        }
        Timer::after(RECONNECT_DELAY).await;
    }
}

/// Samples the RSSI and returns once the link has been too slow for
/// [`STALL_LIMIT`] seconds in a row: the PC is connected, yet the
/// unacknowledged backlog is above [`STALL_BACKLOG`] and not shrinking.
///
/// After a bad radio spell the station can stay connected at a fraction of
/// the throughput, with a normal RSSI and no disconnect event (measured);
/// only a new association brings the speed back. Without a receiver there
/// is nobody to send to, and reconnecting Wi-Fi would not help.
async fn monitor_link(controller: &WifiController<'static>, stack: Stack<'static>) {
    let mut last_unacked = UNACKED.load(Ordering::Relaxed);
    let mut stalled_for = 0;
    let mut config_up_since: Option<Instant> = None;
    loop {
        Timer::after(MONITOR_PERIOD).await;
        if let Ok(rssi) = controller.rssi() {
            RSSI_DBM.store(rssi, Ordering::Relaxed);
        }

        // A backlog is expected while there is no IP yet and shortly after:
        // the first send waits for ARP, which once took 7 s (measured).
        config_up_since = match (stack.is_config_up(), config_up_since) {
            (false, _) => None,
            (true, None) => Some(Instant::now()),
            (true, since) => since,
        };
        let settled = config_up_since.is_some_and(|since| since.elapsed() >= STALL_GRACE);

        let unacked = UNACKED.load(Ordering::Relaxed);
        let falling_behind = unacked > STALL_BACKLOG && unacked >= last_unacked;
        stalled_for = if falling_behind && settled && CONNECTED.load(Ordering::Relaxed) {
            stalled_for + 1
        } else {
            0
        };
        last_unacked = unacked;
        if stalled_for >= STALL_LIMIT {
            return;
        }
    }
}

/// Drives the TCP/IP stack: timers, DHCP, packet I/O.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) -> ! {
    runner.run().await
}

/// Logs IP configuration changes.
#[embassy_executor::task]
async fn link_monitor_task(stack: Stack<'static>) {
    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!("IP address: {}", config.address);
        }
        stack.wait_config_down().await;
        warn!("IP configuration lost");
    }
}
