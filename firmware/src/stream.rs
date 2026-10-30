//! Streams samples to the PC over UDP, see the `protocol` crate.

use core::net::SocketAddrV4;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_net::Stack;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use log::{error, info};
use protocol::{MAX_PACKET_LEN, PacketBuilder};

use crate::acquisition::SAMPLES;

const HOST_ADDR: &str = env!(
    "HOST_ADDR",
    "set HOST_ADDR in firmware/.cargo/secrets.toml, see secrets.toml.example"
);

/// Packets queued in the socket while Wi-Fi is busy: 80 ms of data.
const TX_PACKETS: usize = 4;

pub static PACKETS_SENT: AtomicU32 = AtomicU32::new(0);
/// Packets not handed to the stack, e.g. no IP address yet.
pub static SEND_ERRORS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn stream_task(stack: Stack<'static>) {
    let host: SocketAddrV4 = HOST_ADDR.parse().expect("HOST_ADDR is ip:port");

    // The socket never receives, but the stack needs both buffers.
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0; 64];
    let mut tx_meta = [PacketMetadata::EMPTY; TX_PACKETS];
    let mut tx_buf = [0; TX_PACKETS * MAX_PACKET_LEN];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(0).expect("ephemeral port is free");

    stack.wait_config_up().await;
    info!("Streaming to {host}");

    let mut packet = PacketBuilder::new();
    let mut sequence: u32 = 0;
    loop {
        let sample = SAMPLES.receive().await;

        if !packet.accepts(sample.index) {
            send(&socket, packet.finish(sequence), host).await;
            sequence = sequence.wrapping_add(1);
        }
        packet.push(sample.index, &sample.frame.raw);

        if packet.is_full() {
            send(&socket, packet.finish(sequence), host).await;
            sequence = sequence.wrapping_add(1);
        }
    }
}

async fn send(socket: &UdpSocket<'_>, bytes: &[u8], host: SocketAddrV4) {
    match socket.send_to(bytes, host).await {
        Ok(()) => {
            PACKETS_SENT.fetch_add(1, Ordering::Relaxed);
        }
        Err(e) => {
            if SEND_ERRORS.fetch_add(1, Ordering::Relaxed) == 0 {
                error!("UDP send failed: {e:?}");
            }
        }
    }
}
