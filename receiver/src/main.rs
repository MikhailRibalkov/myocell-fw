//! Finds myocell devices, streams from the chosen one, acknowledges, prints
//! statistics once a second and shows the signal live in a browser.
//!
//! Usage: `cargo run --release [-- <device ip[:port]>]`. The web UI at
//! <http://127.0.0.1:8080> finds devices and connects; the optional address
//! connects right away. Recordings go to `recordings/` in the working
//! directory.

mod device;
mod discovery;
mod live;
mod parser;
mod recorder;
mod spectrum;
mod stats;
mod tracker;
mod web;

use std::io;
use std::net::{IpAddr, SocketAddr};
use std::sync::mpsc;
use std::thread;

use protocol::discovery::DATA_PORT;
use tokio::sync::broadcast;

use crate::device::LiveFeed;
use crate::recorder::Recording;

/// Loopback only: the UI is for this computer, not the network.
const WEB_ADDR: &str = "127.0.0.1:8080";
const RECORDINGS_DIR: &str = "recordings";
/// Batches buffered per browser tab: 64 x 50 ms, about 3 s of signal.
const LIVE_CAPACITY: usize = 64;

#[tokio::main]
async fn main() -> io::Result<()> {
    let target = match std::env::args().nth(1) {
        Some(arg) => Some(parse_device_address(&arg)?),
        None => None,
    };
    // Bound before anything starts, so a busy port fails right here.
    let web_listener = tokio::net::TcpListener::bind(WEB_ADDR).await?;
    println!("web UI: http://{WEB_ADDR}");
    if let Some(target) = target {
        println!("device: connecting to {target}");
    }

    let (updates, _) = broadcast::channel(LIVE_CAPACITY);
    let live = LiveFeed::new(updates.clone());
    let (commands, command_receiver) = mpsc::channel();
    let recording = Recording::new(RECORDINGS_DIR);

    // A thread of its own: its blocking reads must not stall the runtime.
    let device = thread::Builder::new()
        .name("device".into())
        .spawn(move || device::run(live, command_receiver, recording, target))?;
    let device = tokio::task::spawn_blocking(move || device.join());

    // Neither side returns unless it fails; the first failure ends both.
    tokio::select! {
        result = web::serve(web_listener, updates, commands) => result,
        joined = device => joined
            .expect("the join task is never cancelled")
            .expect("device thread panicked"),
    }
}

/// `192.168.1.61` or `192.168.1.61:5005`.
fn parse_device_address(arg: &str) -> io::Result<SocketAddr> {
    let invalid = || {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("not an address: {arg}"),
        )
    };
    arg.parse::<SocketAddr>()
        .or_else(|_| {
            arg.parse::<IpAddr>()
                .map(|ip| SocketAddr::new(ip, DATA_PORT))
        })
        .map_err(|_| invalid())
}
