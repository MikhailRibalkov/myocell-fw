//! The device side: one TCP connection from the board at a time, whose
//! packets are parsed, deduplicated and acknowledged.

use std::io::{self, ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

use tokio::sync::broadcast;

use ads1298::{CHANNELS, Frame, VREF_VOLTS};
use protocol::Ack;
use protocol::setup::{CHANNEL_CONFIG, DATA_RATE};

use crate::live::{Batcher, Update, enabled_channels};
use crate::parser::StreamParser;
use crate::spectrum;
use crate::tracker::Tracker;

const REPORT_PERIOD: Duration = Duration::from_secs(1);
/// How often the device learns what has arrived, so it can free its ring.
const ACK_PERIOD: Duration = Duration::from_millis(100);
/// Upper bound of one blocking read, so acks and reports keep their pace.
const READ_TIMEOUT: Duration = Duration::from_millis(50);
/// A connection without data for this long is dead, e.g. the device lost
/// Wi-Fi and could not even reset the connection.
const DEAD_TIMEOUT: Duration = Duration::from_secs(15);

/// Mains frequency; its harmonics are measured separately.
const MAINS_HZ: f64 = 50.0;
const MAINS_HARMONICS: usize = 5;
/// Peaks are searched up to the Nyquist frequency of 2 kSPS / 2.
const PEAK_SEARCH_MAX_HZ: u32 = 500;

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

    fn push(&mut self, index: u32, microvolts: &[f32; CHANNELS]) {
        let samples = enabled_channels().map(|i| microvolts[i]);
        if let Some(update) = self.batcher.push(index, samples) {
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
        // Accepted sockets of a non-blocking listener may inherit the flag.
        stream.set_nonblocking(false)?;
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
    fn poll(
        &mut self,
        tracker: &mut Tracker,
        window: &mut Window,
        live: &mut LiveFeed,
    ) -> io::Result<()> {
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
            let skip = tracker.on_packet(&packet.header);
            window.packets += 1;
            // Frames of a packet are consecutive conversions; the index
            // wraps like the device counter (`first..` would panic there).
            let first = packet.header.first_frame.wrapping_add(skip as u32);
            for (i, raw) in packet.frames[skip..].iter().enumerate() {
                let index = first.wrapping_add(i as u32);
                match Frame::parse(raw) {
                    Some(frame) => {
                        let microvolts = microvolts(&frame);
                        window.add(&microvolts);
                        live.push(index, &microvolts);
                    }
                    None => window.bad_frames += 1,
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

/// Serves the device: accepts its connections, acknowledges data and
/// reports statistics. Returns only on a listener error.
pub fn run(listener: TcpListener, mut live: LiveFeed) -> io::Result<()> {
    // Polled for new connections between reads: the newest one wins, since
    // a device that reconnects has given up on the old one.
    listener.set_nonblocking(true)?;

    let mut connection: Option<Connection> = None;
    let mut connections = 0;
    let mut tracker = Tracker::default();
    let mut window = Window::new(0);
    let mut next_report = Instant::now() + REPORT_PERIOD;

    loop {
        match listener.accept() {
            Ok((stream, peer)) => {
                if let Some(old) = &connection {
                    println!("{peer} connected, dropping {}", old.peer);
                } else {
                    println!("{peer} connected");
                }
                connection = Some(Connection::new(stream, peer)?);
                connections += 1;
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) => return Err(e),
        }

        match &mut connection {
            Some(c) => {
                if let Err(e) = c.poll(&mut tracker, &mut window, &mut live) {
                    println!("connection from {} lost: {e}", c.peer);
                    connection = None;
                }
            }
            None => std::thread::sleep(READ_TIMEOUT),
        }
        live.send_due();

        if Instant::now() >= next_report {
            report(&window, &tracker, connection.is_some(), connections);
            window = Window::new(tracker.lost_frames);
            next_report += REPORT_PERIOD;
        }
    }
}

fn report(window: &Window, tracker: &Tracker, connected: bool, connections: u32) {
    let gaps = if tracker.lost_frames > window.lost_at_start {
        " | GAPS IN WINDOW"
    } else {
        ""
    };
    println!(
        "{} frames/s in {} packets | lost {} | duplicates {} | bad frames {} | tcp {}, connections {} | restarts {}{gaps}",
        window.frames,
        window.packets,
        tracker.lost_frames,
        tracker.duplicate_frames,
        window.bad_frames,
        if connected { "up" } else { "down" },
        connections,
        tracker.restarts,
    );
    if window.frames == 0 {
        return;
    }
    let sample_rate = f64::from(DATA_RATE.hz());
    let enabled = CHANNEL_CONFIG
        .iter()
        .enumerate()
        .filter(|(_, c)| !c.power_down);
    for (i, _) in enabled {
        let samples = &window.samples[i];
        let n = samples.len() as f64;
        let mean = samples.iter().sum::<f64>() / n;
        let ac: Vec<f64> = samples.iter().map(|x| x - mean).collect();
        let variance = ac.iter().map(|x| x * x).sum::<f64>() / n;

        let harmonics: Vec<f64> = (1..=MAINS_HARMONICS)
            .map(|k| spectrum::tone_rms(&ac, sample_rate, MAINS_HZ * k as f64))
            .collect();
        // Powers of orthogonal components add up, so what is left without
        // the mains harmonics is the rest of the variance.
        let mains_power: f64 = harmonics.iter().map(|rms| rms * rms).sum();
        let rest = (variance - mains_power).max(0.0).sqrt();

        let harmonics_text: Vec<String> = harmonics
            .iter()
            .enumerate()
            .map(|(k, rms)| format!("{}:{rms:.1}", (k + 1) * MAINS_HZ as usize))
            .collect();
        let peaks_text: Vec<String> = spectrum::peaks(&ac, sample_rate, PEAK_SEARCH_MAX_HZ, 3)
            .iter()
            .map(|(f, rms)| format!("{f} Hz {rms:.1}"))
            .collect();
        println!(
            "  CH{}: rms {:7.1} | rest {:7.1} | mean {:+8.1} | mains [{}] | peaks [{}]  (uV)",
            i + 1,
            variance.sqrt(),
            rest,
            mean,
            harmonics_text.join(" "),
            peaks_text.join(", "),
        );
    }
}
