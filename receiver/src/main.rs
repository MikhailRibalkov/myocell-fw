//! Receives the myocell TCP stream, acknowledges it and prints statistics
//! once a second.
//!
//! Usage: `cargo run --release [-- <bind address>]`, default `0.0.0.0:5005`.

mod device;
mod parser;
mod spectrum;
mod tracker;

use std::io;
use std::net::TcpListener;
use std::thread;

const DEFAULT_ADDR: &str = "0.0.0.0:5005";

fn main() -> io::Result<()> {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_owned());
    let listener = TcpListener::bind(&addr)?;
    println!("listening on {addr}");

    // A thread of its own: its blocking reads must not stall the web server
    // that shares this process.
    let device = thread::Builder::new()
        .name("device".into())
        .spawn(move || device::run(listener))?;
    device.join().expect("device thread panicked")
}
