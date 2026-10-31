//! Streams records to the PC over TCP, see the `protocol` crate.
//!
//! The receiver finds the device by discovery (see [`crate::discovery`]) and
//! connects to it; the device sends packets from the backlog ring and the
//! receiver acknowledges what has arrived. Records leave the ring
//! only once acknowledged, and after a reconnect everything unacknowledged
//! is sent again, so nothing is lost while the ring has room.

use core::cell::Cell;
use core::convert::Infallible;
use core::net::IpAddr;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_net::Stack;
use embassy_net::tcp::{TcpReader, TcpSocket, TcpWriter};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use log::{info, warn};
use protocol::discovery::DATA_PORT;
use protocol::{ACK_LEN, Ack, MAX_PACKET_LEN, PacketBuilder};

use crate::acquisition::{BacklogConsumer, NEW_DATA};
use crate::discovery::CLIENT;

/// Socket send buffer: a few packets, the real buffer is the backlog ring.
const TX_BUFFER: usize = 8 * MAX_PACKET_LEN;
/// Without any progress for this long, the connection is considered dead.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(10);
/// Pause after a lost connection before listening again.
const RECONNECT_DELAY: Duration = Duration::from_secs(1);
/// When caught up, a partial packet waits this long for more frames:
/// keeps packets full at 2 kSPS while bounding the latency.
const FLUSH_DELAY: Duration = Duration::from_millis(30);

pub static PACKETS_SENT: AtomicU32 = AtomicU32::new(0);
/// TCP connections that ended with an error and were re-established.
pub static CONNECTION_LOSSES: AtomicU32 = AtomicU32::new(0);
pub static CONNECTED: AtomicBool = AtomicBool::new(false);
/// Longest packet write since the last [`take_send_peak`], in microseconds:
/// long means the socket buffer was full, i.e. Wi-Fi is not keeping up.
static SEND_TIME_PEAK_US: AtomicU32 = AtomicU32::new(0);

/// Returns and resets the longest packet write, in microseconds.
pub fn take_send_peak() -> u32 {
    SEND_TIME_PEAK_US.swap(0, Ordering::Relaxed)
}

#[derive(Debug)]
enum StreamError {
    Tcp(embassy_net::tcp::Error),
    /// The receiver closed the connection.
    Closed,
    BadAck,
}

impl From<embassy_net::tcp::Error> for StreamError {
    fn from(e: embassy_net::tcp::Error) -> Self {
        Self::Tcp(e)
    }
}

#[embassy_executor::task]
pub async fn stream_task(stack: Stack<'static>, mut ring: BacklogConsumer) {
    let mut rx_buffer = [0; 64];
    let mut tx_buffer = [0; TX_BUFFER];
    let mut sequence: u32 = 0;

    loop {
        stack.wait_config_up().await;
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(SOCKET_TIMEOUT));

        // One receiver at a time: while it streams, nothing listens, and
        // discovery reports the device as busy.
        if let Err(e) = socket.accept(DATA_PORT).await {
            warn!("TCP accept on port {DATA_PORT} failed: {e:?}");
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }
        let Some(client) = socket.remote_endpoint() else {
            // Reset right after the handshake.
            socket.abort();
            continue;
        };
        info!(
            "TCP connection from {client}, resending {} records",
            ring.unacked()
        );
        if let IpAddr::V4(ip) = client.addr.into() {
            CLIENT.store(ip.to_bits(), Ordering::Relaxed);
        }
        // Whatever was sent but not acknowledged may have been lost with the
        // previous connection.
        ring.rewind();
        CONNECTED.store(true, Ordering::Relaxed);

        // The reader only records the latest acknowledgement; the writer
        // owns the ring and applies it. Both futures run in this task, so a
        // `Cell` is enough: no locking, no `&mut` shared across `.await`.
        let acked = Cell::new(None);
        let (mut reader, mut writer) = socket.split();
        let error = match select(
            send_loop(&mut writer, &mut ring, &acked, &mut sequence),
            receive_acks(&mut reader, &acked),
        )
        .await
        {
            Either::First(Err(e)) | Either::Second(e) => e,
            Either::First(Ok(never)) => match never {},
        };

        CONNECTED.store(false, Ordering::Relaxed);
        CLIENT.store(0, Ordering::Relaxed);
        CONNECTION_LOSSES.fetch_add(1, Ordering::Relaxed);
        warn!("TCP connection lost: {error:?}");
        socket.abort();
        Timer::after(RECONNECT_DELAY).await;
    }
}

/// Sends packets from the ring until an error ends the connection; it never
/// returns `Ok`, which `Infallible` states in the type.
async fn send_loop(
    writer: &mut TcpWriter<'_>,
    ring: &mut BacklogConsumer,
    acked: &Cell<Option<u32>>,
    sequence: &mut u32,
) -> Result<Infallible, StreamError> {
    let mut packet = PacketBuilder::new();
    loop {
        if let Some(next_frame) = acked.take() {
            // Wrapping difference as signed: correct across u32 overflow.
            ring.ack_while(|record| next_frame.wrapping_sub(record.index) as i32 > 0);
        }

        let Some(record) = ring.peek() else {
            if packet.is_empty() {
                NEW_DATA.wait().await;
            } else if with_timeout(FLUSH_DELAY, NEW_DATA.wait()).await.is_err() {
                send_packet(writer, &mut packet, sequence).await?;
            }
            continue;
        };

        // A gap in the indices ends the packet: frames in a packet are
        // consecutive. The record stays unsent and starts the next packet.
        if !packet.accepts(record.index) {
            send_packet(writer, &mut packet, sequence).await?;
            continue;
        }
        packet.push(record.index, &record.raw);
        ring.mark_sent();
        if packet.is_full() {
            send_packet(writer, &mut packet, sequence).await?;
        }
    }
}

async fn send_packet(
    writer: &mut TcpWriter<'_>,
    packet: &mut PacketBuilder,
    sequence: &mut u32,
) -> Result<(), StreamError> {
    let start = Instant::now();
    writer.write_all(packet.finish(*sequence)).await?;
    let elapsed = start.elapsed().as_micros().try_into().unwrap_or(u32::MAX);
    SEND_TIME_PEAK_US.fetch_max(elapsed, Ordering::Relaxed);

    *sequence = sequence.wrapping_add(1);
    PACKETS_SENT.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

/// Reads acknowledgements until an error ends the connection.
async fn receive_acks(reader: &mut TcpReader<'_>, acked: &Cell<Option<u32>>) -> StreamError {
    let mut buf = [0; ACK_LEN];
    loop {
        match reader.read_exact(&mut buf).await {
            Ok(()) => match Ack::decode(&buf) {
                Ok(ack) => acked.set(Some(ack.next_frame)),
                Err(_) => return StreamError::BadAck,
            },
            Err(embedded_io_async::ReadExactError::UnexpectedEof) => {
                return StreamError::Closed;
            }
            Err(embedded_io_async::ReadExactError::Other(e)) => return e.into(),
        }
    }
}
