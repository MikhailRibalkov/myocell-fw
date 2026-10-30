//! Receives the myocell UDP stream and prints statistics once a second.
//!
//! Usage: `cargo run --release [-- <bind address>]`, default `0.0.0.0:5005`.

mod tracker;

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use ads1298::{CHANNELS, Frame, Gain, VREF_VOLTS};
use tracker::Tracker;

const DEFAULT_ADDR: &str = "0.0.0.0:5005";
const REPORT_PERIOD: Duration = Duration::from_secs(1);

/// Must match `CHANNEL_CONFIG.gain` in the firmware.
const GAIN: Gain = Gain::X1;

/// What arrived during one report period.
struct Window {
    frames: u32,
    bad_frames: u32,
    bad_packets: u32,
    min: [i32; CHANNELS],
    max: [i32; CHANNELS],
}

impl Window {
    fn new() -> Self {
        Self {
            frames: 0,
            bad_frames: 0,
            bad_packets: 0,
            min: [i32::MAX; CHANNELS],
            max: [i32::MIN; CHANNELS],
        }
    }

    fn add(&mut self, frame: &Frame) {
        self.frames += 1;
        for (i, &sample) in frame.samples.iter().enumerate() {
            self.min[i] = self.min[i].min(sample);
            self.max[i] = self.max[i].max(sample);
        }
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
    let uv = |raw| GAIN.to_microvolts(raw, VREF_VOLTS);
    for (i, (&min, &max)) in window.min.iter().zip(&window.max).enumerate() {
        println!("  CH{}: {:+9.1} .. {:+9.1} uV", i + 1, uv(min), uv(max));
    }
}
