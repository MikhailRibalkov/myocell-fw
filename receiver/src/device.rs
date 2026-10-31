//! The device side: a TCP connection to the board chosen in the web UI,
//! whose packets are parsed, deduplicated and acknowledged. The connection
//! is re-established until the user disconnects.

use std::io::{self, ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

use tokio::sync::broadcast;

use ads1298::{CHANNELS, Frame, VREF_VOLTS};
use protocol::Ack;
use protocol::setup::{CHANNEL_CONFIG, DATA_RATE};

use crate::live::{Batcher, Update, enabled_channels};
use crate::parser::StreamParser;
use crate::recorder::{self, Recording};
use crate::stats::{ChannelStats, LinkStats};
use crate::tracker::Tracker;

const REPORT_PERIOD: Duration = Duration::from_secs(1);
/// How often the device learns what has arrived, so it can free its ring.
const ACK_PERIOD: Duration = Duration::from_millis(100);
/// Upper bound of one blocking read, so acks and reports keep their pace.
const READ_TIMEOUT: Duration = Duration::from_millis(50);
/// A connection without data for this long is dead, e.g. the device lost
/// Wi-Fi and could not even reset the connection.
const DEAD_TIMEOUT: Duration = Duration::from_secs(15);
/// The device is on the local network: it answers within milliseconds or
/// not at all.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
/// Between attempts to reach the chosen device. Also covers the second the
/// device pauses before listening again after a lost connection.
const RETRY_DELAY: Duration = Duration::from_secs(2);

/// What the web UI asks the device thread to do.
#[derive(Clone, Debug)]
pub enum Command {
    Record(recorder::Command),
    /// Stream from the device at this address, reconnecting until
    /// [`Command::Disconnect`].
    Connect(SocketAddr),
    Disconnect,
}

/// What arrived during one report period. Samples are in microvolts.
struct Window {
    packets: u32,
    frames: u32,
    bad_frames: u32,
    /// Lost-frame counter when the window started: a gap inside the window
    /// breaks its time base, so its spectrum is less reliable.
    lost_at_start: u64,
    samples: [Vec<f64>; CHANNELS],
}

impl Window {
    fn new(lost_at_start: u64) -> Self {
        Self {
            packets: 0,
            frames: 0,
            bad_frames: 0,
            lost_at_start,
            samples: Default::default(),
        }
    }

    fn add(&mut self, microvolts: &[f32; CHANNELS]) {
        self.frames += 1;
        for (&uv, samples) in microvolts.iter().zip(&mut self.samples) {
            samples.push(f64::from(uv));
        }
    }
}

/// Converts with the gain each channel was configured with.
fn microvolts(frame: &Frame) -> [f32; CHANNELS] {
    let mut out = [0.0; CHANNELS];
    for ((out, &raw), config) in out.iter_mut().zip(&frame.samples).zip(&CHANNEL_CONFIG) {
        *out = config.gain.to_microvolts(raw, VREF_VOLTS);
    }
    out
}

/// Forwards the samples of the enabled channels to the web UI.
pub struct LiveFeed {
    batcher: Batcher,
    sender: broadcast::Sender<Update>,
}

impl LiveFeed {
    pub fn new(sender: broadcast::Sender<Update>) -> Self {
        Self {
            batcher: Batcher::new(enabled_channels().count()),
            sender,
        }
    }

    /// `samples` are the enabled channels in microvolts.
    fn push(&mut self, index: u32, samples: &[f32]) {
        if let Some(update) = self.batcher.push(index, samples.iter().copied()) {
            self.send(update);
        }
    }

    fn send_due(&mut self) {
        if let Some(update) = self.batcher.take_due() {
            self.send(update);
        }
    }

    fn send(&self, update: Update) {
        // Fails only when no browser is connected: then nobody needs it.
        let _ = self.sender.send(update);
    }
}

/// Everything a received frame goes to.
struct Sinks {
    window: Window,
    live: LiveFeed,
    recording: Recording,
}

/// One TCP connection from the device.
struct Connection {
    stream: TcpStream,
    peer: SocketAddr,
    parser: StreamParser,
    last_ack: Instant,
    last_data: Instant,
}

impl Connection {
    fn new(stream: TcpStream, peer: SocketAddr) -> io::Result<Self> {
        stream.set_read_timeout(Some(READ_TIMEOUT))?;
        // Acks are tiny: send them now instead of waiting to batch them.
        stream.set_nodelay(true)?;
        let now = Instant::now();
        Ok(Self {
            stream,
            peer,
            parser: StreamParser::default(),
            last_ack: now,
            last_data: now,
        })
    }

    /// Reads what has arrived, processes whole packets and acknowledges.
    /// An error ends the connection.
    fn poll(&mut self, tracker: &mut Tracker, sinks: &mut Sinks) -> io::Result<()> {
        let mut buf = [0; 64 * 1024];
        match self.stream.read(&mut buf) {
            Ok(0) => {
                return Err(io::Error::new(
                    ErrorKind::UnexpectedEof,
                    "closed by the device",
                ));
            }
            Ok(n) => {
                self.parser.extend(&buf[..n]);
                self.last_data = Instant::now();
            }
            // The read timeout shows up as one of these, depending on the OS.
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(e) => return Err(e),
        }
        if self.last_data.elapsed() > DEAD_TIMEOUT {
            return Err(io::Error::new(ErrorKind::TimedOut, "no data"));
        }

        while let Some(packet) = self.parser.next_packet() {
            let packet =
                packet.map_err(|e| io::Error::new(ErrorKind::InvalidData, format!("{e:?}")))?;
            let restarts = tracker.restarts;
            let skip = tracker.on_packet(&packet.header);
            if tracker.restarts != restarts {
                // Before the first frame of the new session is written.
                sinks.recording.split();
            }
            sinks.window.packets += 1;
            // Frames of a packet are consecutive conversions; the index
            // wraps like the device counter (`first..` would panic there).
            let first = packet.header.first_frame.wrapping_add(skip as u32);
            for (i, raw) in packet.frames[skip..].iter().enumerate() {
                let index = first.wrapping_add(i as u32);
                match Frame::parse(raw) {
                    Some(frame) => {
                        let microvolts = microvolts(&frame);
                        sinks.window.add(&microvolts);
                        let enabled: Vec<f32> = enabled_channels().map(|i| microvolts[i]).collect();
                        sinks.live.push(index, &enabled);
                        sinks.recording.write(index, &enabled);
                    }
                    None => sinks.window.bad_frames += 1,
                }
            }
        }

        if self.last_ack.elapsed() >= ACK_PERIOD {
            if let Some(next_frame) = tracker.next_frame() {
                self.stream.write_all(&Ack { next_frame }.encode())?;
            }
            self.last_ack = Instant::now();
        }
        Ok(())
    }
}

/// The device the user chose and how the connection to it is going.
struct Link {
    target: Option<SocketAddr>,
    connection: Option<Connection>,
    next_attempt: Instant,
    /// Why the last attempt failed; printed once, not every retry.
    error: Option<String>,
    connections: u32,
}

impl Link {
    fn status(&self) -> Update {
        Update::Connection {
            target: self.target.map(|t| t.to_string()),
            connected: self.connection.is_some(),
            error: self.error.clone(),
        }
    }

    /// Returns whether the status changed.
    fn handle(&mut self, command: &Command) -> bool {
        match *command {
            Command::Connect(target) => {
                if self.target != Some(target) {
                    self.connection = None;
                }
                self.target = Some(target);
                self.error = None;
                self.next_attempt = Instant::now();
            }
            Command::Disconnect => {
                if let Some(target) = self.target.take() {
                    println!("disconnected from {target}");
                }
                self.connection = None;
                self.error = None;
            }
            Command::Record(_) => return false,
        }
        true
    }

    /// Tries to connect if a device is chosen, not connected and due.
    /// Returns whether the status changed.
    fn connect_due(&mut self) -> io::Result<bool> {
        let Some(target) = self.target else {
            return Ok(false);
        };
        if self.connection.is_some() || Instant::now() < self.next_attempt {
            return Ok(false);
        }
        // Blocks this thread for up to CONNECT_TIMEOUT: nothing streams
        // meanwhile anyway, and commands wait in their channel.
        match TcpStream::connect_timeout(&target, CONNECT_TIMEOUT) {
            Ok(stream) => {
                println!("connected to {target}");
                self.connection = Some(Connection::new(stream, target)?);
                self.connections += 1;
                self.error = None;
                Ok(true)
            }
            Err(e) => {
                self.next_attempt = Instant::now() + RETRY_DELAY;
                let error = e.to_string();
                let changed = self.error.as_ref() != Some(&error);
                if changed {
                    println!("cannot connect to {target}: {error}, retrying");
                }
                self.error = Some(error);
                Ok(changed)
            }
        }
    }
}

/// Serves the chosen device: connects, acknowledges data, records on
/// command and reports statistics. Returns when the web server is gone.
pub fn run(
    live: LiveFeed,
    commands: Receiver<Command>,
    recording: Recording,
    target: Option<SocketAddr>,
) -> io::Result<()> {
    let mut link = Link {
        target,
        connection: None,
        next_attempt: Instant::now(),
        error: None,
        connections: 0,
    };
    let mut tracker = Tracker::default();
    let mut sinks = Sinks {
        window: Window::new(0),
        live,
        recording,
    };
    let mut next_report = Instant::now() + REPORT_PERIOD;

    loop {
        // Commands are rare and the loop turns at least every 50 ms
        // (`READ_TIMEOUT`), so polling them here is quick enough.
        loop {
            match commands.try_recv() {
                Ok(Command::Record(command)) => {
                    if let Some(update) = sinks.recording.handle(command) {
                        sinks.live.send(update);
                    }
                    sinks.live.send(sinks.recording.status());
                }
                Ok(command) => {
                    if link.handle(&command) {
                        sinks.live.send(link.status());
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return Ok(()),
            }
        }

        if link.connect_due()? {
            sinks.live.send(link.status());
        }

        match &mut link.connection {
            Some(c) => {
                if let Err(e) = c.poll(&mut tracker, &mut sinks) {
                    println!("connection to {} lost: {e}", c.peer);
                    link.connection = None;
                    link.next_attempt = Instant::now() + RETRY_DELAY;
                    sinks.live.send(link.status());
                }
            }
            None => std::thread::sleep(READ_TIMEOUT),
        }
        sinks.live.send_due();
        sinks.recording.flush_due();

        if Instant::now() >= next_report {
            let update = report(
                &sinks.window,
                &tracker,
                link.connection.is_some(),
                link.connections,
            );
            sinks.live.send(update);
            sinks.live.send(link.status());
            sinks.window = Window::new(tracker.lost_frames);
            sinks.live.send(sinks.recording.status());
            next_report += REPORT_PERIOD;
        }
    }
}

/// Prints the statistics of the period and returns them for the web UI.
fn report(window: &Window, tracker: &Tracker, connected: bool, connections: u32) -> Update {
    let link = LinkStats {
        frames: window.frames,
        packets: window.packets,
        bad_frames: window.bad_frames,
        lost_frames: tracker.lost_frames,
        duplicate_frames: tracker.duplicate_frames,
        restarts: tracker.restarts,
        connected,
        connections,
        gaps: tracker.lost_frames > window.lost_at_start,
    };
    println!("{}", link.console_line());

    let channels = if window.frames == 0 {
        Vec::new()
    } else {
        let sample_rate = f64::from(DATA_RATE.hz());
        enabled_channels()
            .map(|i| {
                const MAX_CODE: i32 = (1 << 23) - 1;
                let full_scale = CHANNEL_CONFIG[i].gain.to_microvolts(MAX_CODE, VREF_VOLTS);
                let stats = ChannelStats::new(
                    format!("CH{}", i + 1),
                    &window.samples[i],
                    sample_rate,
                    f64::from(full_scale),
                );
                println!("{}", stats.console_line());
                stats
            })
            .collect()
    };
    Update::Stats { link, channels }
}
