//! Finds devices on the local networks, see `protocol::discovery`.

use std::collections::BTreeMap;
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::time::Duration;

use protocol::discovery::{Announce, DISCOVERY_PORT, MAX_ANNOUNCE_LEN, request};
use serde::Serialize;
use tokio::net::UdpSocket;
use tokio::time::{Instant, timeout_at};

/// How long answers are collected. A device answers within milliseconds,
/// unless its Wi-Fi is dozing in power save.
const LISTEN: Duration = Duration::from_millis(800);
/// UDP may drop the broadcast, so it is sent once more after this.
const RESEND_AFTER: Duration = Duration::from_millis(300);

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Device {
    /// `myocell-` and the last three bytes of the MAC, as printed nowhere
    /// else but stable for the board.
    pub name: String,
    pub mac: String,
    /// Where to connect: the address the answer came from.
    pub address: SocketAddr,
    /// The receiver it streams to, if any.
    pub client: Option<Ipv4Addr>,
    pub firmware: String,
}

impl Device {
    fn new(announce: &Announce, from: SocketAddrV4) -> Self {
        let mac = announce.mac;
        Self {
            name: format!("myocell-{:02x}{:02x}{:02x}", mac[3], mac[4], mac[5]),
            mac: mac.map(|b| format!("{b:02x}")).join(":"),
            address: SocketAddr::V4(SocketAddrV4::new(*from.ip(), announce.data_port)),
            client: announce.client,
            firmware: announce.firmware.to_owned(),
        }
    }
}

/// Directed broadcast addresses of the IPv4 interfaces that are up. macOS
/// refuses the limited broadcast 255.255.255.255 ("No route to host"), and
/// it would leave through one interface only anyway.
fn broadcast_addresses() -> io::Result<Vec<Ipv4Addr>> {
    let mut addresses: Vec<Ipv4Addr> = if_addrs::get_if_addrs()?
        .into_iter()
        .filter(|interface| !interface.is_loopback())
        .filter_map(|interface| match interface.addr {
            if_addrs::IfAddr::V4(v4) => v4.broadcast,
            _ => None,
        })
        .collect();
    addresses.sort();
    addresses.dedup();
    Ok(addresses)
}

async fn ask(socket: &UdpSocket, targets: &[Ipv4Addr]) {
    for &target in targets {
        // One network refusing (a VPN, a link going down) must not hide
        // the devices on the others.
        if let Err(e) = socket.send_to(&request(), (target, DISCOVERY_PORT)).await {
            println!("discovery: broadcast to {target} failed: {e}");
        }
    }
}

/// Broadcasts a request on every network and collects the answers.
pub async fn discover() -> io::Result<Vec<Device>> {
    let targets = broadcast_addresses()?;
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
    socket.set_broadcast(true)?;
    ask(&socket, &targets).await;

    let start = Instant::now();
    let mut resent = false;
    // By MAC: a device answers each request, and may be reachable twice.
    let mut found = BTreeMap::new();
    // One byte more than the longest announce: anything longer is foreign.
    let mut buf = [0; MAX_ANNOUNCE_LEN + 1];
    loop {
        let deadline = start + if resent { LISTEN } else { RESEND_AFTER };
        match timeout_at(deadline, socket.recv_from(&mut buf)).await {
            Ok(Ok((len, SocketAddr::V4(from)))) => {
                if let Ok(announce) = Announce::decode(&buf[..len]) {
                    found.insert(announce.mac, Device::new(&announce, from));
                }
            }
            // IPv6 cannot answer an IPv4 broadcast; errors such as an ICMP
            // "port unreachable" from another host are not fatal either.
            Ok(Ok(_)) | Ok(Err(_)) => {}
            Err(_timeout) if !resent => {
                ask(&socket, &targets).await;
                resent = true;
            }
            Err(_timeout) => break,
        }
    }
    Ok(found.into_values().collect())
}
