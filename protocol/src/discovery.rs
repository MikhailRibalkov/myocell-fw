//! UDP discovery: the PC broadcasts a request to [`DISCOVERY_PORT`], every
//! device answers the sender with an [`Announce`]; the PC then opens the
//! data stream itself, to [`Announce::data_port`] on the answering address.
//!
//! Request, [`REQUEST_LEN`] bytes: magic "MYOD", version.
//!
//! Announce:
//!
//! ```text
//! offset size
//! 0      4    magic "MYOI"
//! 4      1    version
//! 5      6    MAC address of the Wi-Fi station
//! 11     2    data_port    u16 LE, TCP
//! 13     4    client       IPv4 of the connected receiver, 0.0.0.0 if free
//! 17     1    N            length of the firmware version, <= MAX_FIRMWARE_LEN
//! 18     N    firmware     UTF-8
//! ```
//!
//! The device's own IP address is not in the message: the PC takes it from
//! the UDP source address, which is the one that is reachable.

use core::net::Ipv4Addr;

use crate::DecodeError;

pub const DISCOVERY_PORT: u16 = 5006;
pub const DATA_PORT: u16 = 5005;
pub const VERSION: u8 = 1;

const REQUEST_MAGIC: [u8; 4] = *b"MYOD";
const ANNOUNCE_MAGIC: [u8; 4] = *b"MYOI";
pub const REQUEST_LEN: usize = 5;
const ANNOUNCE_HEADER_LEN: usize = 18;
pub const MAX_FIRMWARE_LEN: usize = 16;
pub const MAX_ANNOUNCE_LEN: usize = ANNOUNCE_HEADER_LEN + MAX_FIRMWARE_LEN;

pub const fn request() -> [u8; REQUEST_LEN] {
    let mut out = [0; REQUEST_LEN];
    let mut i = 0;
    while i < 4 {
        out[i] = REQUEST_MAGIC[i];
        i += 1;
    }
    out[4] = VERSION;
    out
}

/// Anything else on the port (another protocol, another version) is not
/// answered.
pub fn is_request(bytes: &[u8]) -> bool {
    bytes == request()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Announce<'a> {
    pub mac: [u8; 6],
    pub data_port: u16,
    /// The receiver streaming from the device; `None` if it is free.
    pub client: Option<Ipv4Addr>,
    /// Cut to [`MAX_FIRMWARE_LEN`] bytes when encoded.
    pub firmware: &'a str,
}

impl<'a> Announce<'a> {
    /// Returns the length written to `out`.
    pub fn encode(&self, out: &mut [u8; MAX_ANNOUNCE_LEN]) -> usize {
        // Cut at a character boundary, so the version stays valid UTF-8.
        let mut len = self.firmware.len().min(MAX_FIRMWARE_LEN);
        while !self.firmware.is_char_boundary(len) {
            len -= 1;
        }
        out[0..4].copy_from_slice(&ANNOUNCE_MAGIC);
        out[4] = VERSION;
        out[5..11].copy_from_slice(&self.mac);
        out[11..13].copy_from_slice(&self.data_port.to_le_bytes());
        let client = self.client.unwrap_or(Ipv4Addr::UNSPECIFIED);
        out[13..17].copy_from_slice(&client.octets());
        out[17] = len as u8;
        out[18..18 + len].copy_from_slice(&self.firmware.as_bytes()[..len]);
        ANNOUNCE_HEADER_LEN + len
    }

    pub fn decode(bytes: &'a [u8]) -> Result<Self, DecodeError> {
        let header: &[u8; ANNOUNCE_HEADER_LEN] =
            bytes.first_chunk().ok_or(DecodeError::TooShort)?;
        if header[0..4] != ANNOUNCE_MAGIC || header[4] != VERSION {
            return Err(DecodeError::BadMagic);
        }
        let len = usize::from(header[17]);
        if len > MAX_FIRMWARE_LEN || bytes.len() != ANNOUNCE_HEADER_LEN + len {
            return Err(DecodeError::LengthMismatch);
        }
        let firmware = core::str::from_utf8(&bytes[ANNOUNCE_HEADER_LEN..])
            .map_err(|_| DecodeError::LengthMismatch)?;
        let client = Ipv4Addr::from(<[u8; 4]>::try_from(&header[13..17]).unwrap());
        Ok(Self {
            mac: header[5..11].try_into().unwrap(),
            data_port: u16::from_le_bytes([header[11], header[12]]),
            client: (!client.is_unspecified()).then_some(client),
            firmware,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAC: [u8; 6] = [0xb4, 0x3a, 0x45, 0x0c, 0x2f, 0xfc];

    fn roundtrip(announce: Announce) {
        let mut buf = [0; MAX_ANNOUNCE_LEN];
        let len = announce.encode(&mut buf);
        assert_eq!(Announce::decode(&buf[..len]), Ok(announce));
    }

    #[test]
    fn announces_free_and_busy_devices() {
        roundtrip(Announce {
            mac: MAC,
            data_port: DATA_PORT,
            client: None,
            firmware: "0.1.0",
        });
        roundtrip(Announce {
            mac: MAC,
            data_port: 7000,
            client: Some(Ipv4Addr::new(192, 168, 1, 138)),
            firmware: "",
        });
    }

    #[test]
    fn cuts_long_firmware_versions_at_a_character() {
        let mut buf = [0; MAX_ANNOUNCE_LEN];
        // 15 ASCII bytes, then a 2-byte character that would end at 17.
        let firmware = "0.1.0-abcdefghiю";
        let len = Announce {
            mac: MAC,
            data_port: 1,
            client: None,
            firmware,
        }
        .encode(&mut buf);
        let decoded = Announce::decode(&buf[..len]).unwrap();
        assert_eq!(decoded.firmware, "0.1.0-abcdefghi");
    }

    #[test]
    fn rejects_foreign_or_broken_messages() {
        let mut buf = [0; MAX_ANNOUNCE_LEN];
        let len = Announce {
            mac: MAC,
            data_port: 1,
            client: None,
            firmware: "1",
        }
        .encode(&mut buf);
        assert_eq!(
            Announce::decode(&buf[..len - 1]),
            Err(DecodeError::LengthMismatch)
        );
        assert_eq!(Announce::decode(&buf[..10]), Err(DecodeError::TooShort));
        buf[4] = VERSION + 1;
        assert_eq!(Announce::decode(&buf[..len]), Err(DecodeError::BadMagic));
    }

    #[test]
    fn recognizes_only_its_own_requests() {
        assert!(is_request(&request()));
        assert!(!is_request(b"MYOD"));
        assert!(!is_request(b"MYOD\x02"));
        assert!(!is_request(b"MYODx\x01"));
    }
}
