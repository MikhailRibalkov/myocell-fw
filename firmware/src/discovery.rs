//! Answers discovery broadcasts from receivers, see `protocol::discovery`.
//!
//! The receiver finds the device this way and then connects to it, so the
//! firmware needs no PC address.

use core::net::Ipv4Addr;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{HardwareAddress, Stack};
use log::{info, warn};
use protocol::discovery::{
    Announce, DATA_PORT, DISCOVERY_PORT, MAX_ANNOUNCE_LEN, REQUEST_LEN, is_request,
};

/// IPv4 of the receiver streaming from the device, `0` while free; set by
/// the stream task, announced here.
pub static CLIENT: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn discovery_task(stack: Stack<'static>) {
    let HardwareAddress::Ethernet(mac) = stack.hardware_address();

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buffer = [0; 4 * REQUEST_LEN];
    let mut tx_buffer = [0; 4 * MAX_ANNOUNCE_LEN];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket
        .bind(DISCOVERY_PORT)
        .expect("one socket per port, and this is the only one");
    info!("Discovery: answering on UDP port {DISCOVERY_PORT}");

    // One byte more than a request: a longer datagram is not one.
    let mut request = [0; REQUEST_LEN + 1];
    let mut announce = [0; MAX_ANNOUNCE_LEN];
    loop {
        let (len, meta) = match socket.recv_from(&mut request).await {
            Ok(received) => received,
            // Truncated: too long to be a request.
            Err(_) => continue,
        };
        if !is_request(&request[..len]) {
            continue;
        }
        let client = Ipv4Addr::from_bits(CLIENT.load(Ordering::Relaxed));
        let len = Announce {
            mac: mac.0,
            data_port: DATA_PORT,
            client: (!client.is_unspecified()).then_some(client),
            firmware: env!("CARGO_PKG_VERSION"),
        }
        .encode(&mut announce);
        // To the asker only: no broadcast storm with several receivers.
        if let Err(e) = socket.send_to(&announce[..len], meta.endpoint).await {
            warn!("Discovery: reply to {} failed: {e:?}", meta.endpoint);
        }
    }
}
