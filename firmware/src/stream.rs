//! Streams samples to the PC over UDP, see the `protocol` crate.

use core::net::SocketAddrV4;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_net::Stack;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_time::Instant;
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
/// Longest `send_to` since the last [`take_peaks`], in microseconds: long
/// means the socket buffer was full, i.e. Wi-Fi is not keeping up.
static SEND_TIME_PEAK_US: AtomicU32 = AtomicU32::new(0);
/// Highest [`SAMPLES`] fill level since the last [`take_peaks`].
static QUEUE_PEAK: AtomicU32 = AtomicU32::new(0);
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
    // The queue overflowed while Wi-Fi was connecting: drop the stale frames
    // so that the stream starts with fresh, consecutive data.
    SAMPLES.clear();
    info!("Streaming to {host}");

    let mut packet = PacketBuilder::new();
    let mut sequence: u32 = 0;
    loop {
        let sample = SAMPLES.receive().await;
        // +1: the sample just taken was in the queue too.
        QUEUE_PEAK.fetch_max(SAMPLES.len() as u32 + 1, Ordering::Relaxed);

        if !packet.accepts(sample.index) {
            send(&socket, packet.finish(sequence), host).await;
            sequence = sequence.wrapping_add(1);
        }
        packet.push(sample.index, &sample.raw);

        if packet.is_full() {
            send(&socket, packet.finish(sequence), host).await;
            sequence = sequence.wrapping_add(1);
        }
    }
}

/// Returns and resets the peaks: `(longest send in us, queue fill level)`.
pub fn take_peaks() -> (u32, u32) {
    (
        SEND_TIME_PEAK_US.swap(0, Ordering::Relaxed),
        QUEUE_PEAK.swap(0, Ordering::Relaxed),
    )
}

async fn send(socket: &UdpSocket<'_>, bytes: &[u8], host: SocketAddrV4) {
    let start = Instant::now();
    let result = socket.send_to(bytes, host).await;
    let elapsed = start.elapsed().as_micros().try_into().unwrap_or(u32::MAX);
    SEND_TIME_PEAK_US.fetch_max(elapsed, Ordering::Relaxed);

    match result {
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
