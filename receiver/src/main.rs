//! Receives the myocell UDP stream and prints statistics once a second.
//!
//! Usage: `cargo run --release [-- <bind address>]`, default `0.0.0.0:5005`.

mod spectrum;
mod tracker;

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use ads1298::{CHANNELS, Frame, VREF_VOLTS};
use protocol::setup::{CHANNEL_CONFIG, DATA_RATE};
use tracker::Tracker;

const DEFAULT_ADDR: &str = "0.0.0.0:5005";
const REPORT_PERIOD: Duration = Duration::from_secs(1);

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
    bad_packets: u32,
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
            bad_packets: 0,
            lost_at_start,
            samples: Default::default(),
        }
    }

    fn add(&mut self, frame: &Frame) {
        self.frames += 1;
        for ((&raw, config), samples) in frame
            .samples
            .iter()
            .zip(&CHANNEL_CONFIG)
            .zip(&mut self.samples)
        {
            samples.push(f64::from(config.gain.to_microvolts(raw, VREF_VOLTS)));
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
    let mut window = Window::new(0);
    let mut next_report = Instant::now() + REPORT_PERIOD;

    loop {
        match socket.recv(&mut buf) {
            Ok(len) => match protocol::decode(&buf[..len]) {
                Ok((header, frames)) => {
                    if tracker.on_packet(&header) {
                        window.packets += 1;
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
            window = Window::new(tracker.lost_frames);
            next_report += REPORT_PERIOD;
        }
    }
}

fn report(window: &Window, tracker: &Tracker) {
    let gaps = if tracker.lost_frames > window.lost_at_start {
        " | GAPS IN WINDOW"
    } else {
        ""
    };
    println!(
        "{} frames/s in {} packets | lost: {} frames, {} packets | late {} | bad: {} packets, {} frames | restarts {}{gaps}",
        window.frames,
        window.packets,
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
