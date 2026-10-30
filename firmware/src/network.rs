//! Wi-Fi station and the embassy-net TCP/IP stack.
//!
//! Credentials come from `.cargo/secrets.toml` (see `secrets.toml.example`)
//! and are baked in at compile time.

use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{AuthenticationMethodConfig, Config, Interface, WifiController};
use log::{info, warn};
use static_cell::StaticCell;

const SSID: &str = env!(
    "WIFI_SSID",
    "set WIFI_SSID in firmware/.cargo/secrets.toml, see secrets.toml.example"
);
const PASSWORD: &str = env!(
    "WIFI_PASSWORD",
    "set WIFI_PASSWORD in firmware/.cargo/secrets.toml, see secrets.toml.example"
);

const RECONNECT_DELAY: Duration = Duration::from_secs(2);

/// DHCP client, the data UDP socket and one spare.
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

    spawner.spawn(connection_task(controller).expect("task pool is free"));
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
async fn connection_task(mut controller: WifiController<'static>) {
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
                let reason = controller.wait_for_disconnect_async().await;
                warn!("Wi-Fi disconnected: {reason:?}");
            }
            Err(e) => warn!("Wi-Fi connect failed: {e:?}"),
        }
        Timer::after(RECONNECT_DELAY).await;
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
