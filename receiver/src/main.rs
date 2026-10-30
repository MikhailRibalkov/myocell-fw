//! Receives the myocell TCP stream, acknowledges it, prints statistics once
//! a second and shows the signal live in a browser.
//!
//! Usage: `cargo run --release [-- <device bind address>]`, default
//! `0.0.0.0:5005`; the web UI is at <http://127.0.0.1:8080>. Recordings go
//! to `recordings/` in the working directory.

mod device;
mod live;
mod parser;
mod recorder;
mod spectrum;
mod tracker;
mod web;

use std::io;
use std::sync::mpsc;
use std::thread;

use tokio::sync::broadcast;

use crate::device::LiveFeed;
use crate::recorder::Recording;

const DEFAULT_DEVICE_ADDR: &str = "0.0.0.0:5005";
/// Loopback only: the UI is for this computer, not the network.
const WEB_ADDR: &str = "127.0.0.1:8080";
const RECORDINGS_DIR: &str = "recordings";
/// Batches buffered per browser tab: 64 x 50 ms, about 3 s of signal.
const LIVE_CAPACITY: usize = 64;

#[tokio::main]
async fn main() -> io::Result<()> {
    let device_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_DEVICE_ADDR.to_owned());
    // Both bound before anything starts, so a busy port fails right here.
    let device_listener = std::net::TcpListener::bind(&device_addr)?;
    let web_listener = tokio::net::TcpListener::bind(WEB_ADDR).await?;
    println!("device: listening on {device_addr}");
    println!("web UI: http://{WEB_ADDR}");

    let (updates, _) = broadcast::channel(LIVE_CAPACITY);
    let live = LiveFeed::new(updates.clone());
    let (commands, command_receiver) = mpsc::channel();
    let recording = Recording::new(RECORDINGS_DIR);

    // A thread of its own: its blocking reads must not stall the runtime.
    let device = thread::Builder::new()
        .name("device".into())
        .spawn(move || device::run(device_listener, live, command_receiver, recording))?;
    let device = tokio::task::spawn_blocking(move || device.join());

    // Neither side returns unless it fails; the first failure ends both.
    tokio::select! {
        result = web::serve(web_listener, updates, commands) => result,
        joined = device => joined
            .expect("the join task is never cancelled")
            .expect("device thread panicked"),
    }
}
