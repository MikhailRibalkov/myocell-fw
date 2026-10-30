//! Receives the myocell UDP stream and prints statistics once a second.
//!
//! Usage: `cargo run --release [-- <bind address>]`, default `0.0.0.0:5005`.

mod tracker;

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use ads1298::{CHANNELS, Frame, VREF_VOLTS};
use protocol::setup::CHANNEL_CONFIG;
use tracker::Tracker;

const DEFAULT_ADDR: &str = "0.0.0.0:5005";
const REPORT_PERIOD: Duration = Duration::from_secs(1);

/// What arrived during one report period. Samples are in microvolts.
struct Window {
    frames: u32,
    bad_frames: u32,
    bad_packets: u32,
    sum: [f64; CHANNELS],
    sum_sq: [f64; CHANNELS],
    min: [f64; CHANNELS],
    max: [f64; CHANNELS],
}

impl Window {
    fn new() -> Self {
        Self {
            frames: 0,
            bad_frames: 0,
            bad_packets: 0,
            sum: [0.0; CHANNELS],
            sum_sq: [0.0; CHANNELS],
            min: [f64::INFINITY; CHANNELS],
            max: [f64::NEG_INFINITY; CHANNELS],
        }
    }

    fn add(&mut self, frame: &Frame) {
        self.frames += 1;
        for (i, (&raw, config)) in frame.samples.iter().zip(&CHANNEL_CONFIG).enumerate() {
            let uv = f64::from(config.gain.to_microvolts(raw, VREF_VOLTS));
            self.sum[i] += uv;
            self.sum_sq[i] += uv * uv;
            self.min[i] = self.min[i].min(uv);
            self.max[i] = self.max[i].max(uv);
        }
    }

    /// Mean and RMS of the AC part (standard deviation) of channel `i`.
    fn mean_and_rms(&self, i: usize) -> (f64, f64) {
        let n = f64::from(self.frames);
        let mean = self.sum[i] / n;
        // Clamp: rounding can make the variance slightly negative.
        let variance = (self.sum_sq[i] / n - mean * mean).max(0.0);
        (mean, variance.sqrt())
    }
}

fn main() -> std::io::Result<()> {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_owned());
    let socket = UdpSocket::bind(&addr)?;
    // Wake up regularly even when nothing arrives, so silence gets reported.
    socket.set_read_timeout(Some(REPORT_PERIOD))?;
    println!("listening on {addr}");

    let mut buf = [0; 2048];
    let mut tracker = Tracker::default();
    let mut window = Window::new();
    let mut next_report = Instant::now() + REPORT_PERIOD;

    loop {
        match socket.recv(&mut buf) {
            Ok(len) => match protocol::decode(&buf[..len]) {
                Ok((header, frames)) => {
                    if tracker.on_packet(&header) {
                        for raw in frames {
                            match Frame::parse(raw) {
                                Some(frame) => window.add(&frame),
                                None => window.bad_frames += 1,
                            }
                        }
                    }
                }
                Err(_) => window.bad_packets += 1,
            },
            // The read timeout shows up as one of these, depending on the OS.
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(e) => return Err(e),
        }

        if Instant::now() >= next_report {
            report(&window, &tracker);
            window = Window::new();
            next_report += REPORT_PERIOD;
        }
    }
}

fn report(window: &Window, tracker: &Tracker) {
    println!(
        "{} frames/s | lost: {} frames, {} packets | late {} | bad: {} packets, {} frames | restarts {}",
        window.frames,
        tracker.lost_frames,
        tracker.lost_packets,
        tracker.late_packets,
        window.bad_packets,
        window.bad_frames,
        tracker.restarts,
    );
    if window.frames == 0 {
        return;
    }
    let enabled = CHANNEL_CONFIG
        .iter()
        .enumerate()
        .filter(|(_, c)| !c.power_down);
    for (i, _) in enabled {
        let (mean, rms) = window.mean_and_rms(i);
        println!(
            "  CH{}: rms {:8.1} uV | mean {:+9.1} uV | p-p {:8.1} uV",
            i + 1,
            rms,
            mean,
            window.max[i] - window.min[i],
        );
    }
}
