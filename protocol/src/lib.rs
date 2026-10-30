//! UDP stream format between the myocell firmware and the PC receiver.
//!
//! ```text
//! offset size
//! 0      4    magic "MYO1"
//! 4      4    sequence     u32 LE, packet counter: gaps = network loss
//! 8      4    first_frame  u32 LE, conversion index of the first frame
//! 12     2    frame_count  u16 LE, 1..=FRAMES_PER_PACKET
//! 14     27*N raw ADS1298 frames, consecutive conversions
//! ```
//!
//! Conversion indices count every conversion of the ADC, including the ones
//! the device failed to read, so any gap in `first_frame` is lost data.

#![no_std]

pub mod setup;

pub use ads1298::FRAME_LEN;

pub const MAGIC: [u8; 4] = *b"MYO1";
pub const HEADER_LEN: usize = 14;
/// 20 ms of data at 2 kSPS; the packet stays below a 1500-byte MTU.
pub const FRAMES_PER_PACKET: usize = 40;
pub const MAX_PACKET_LEN: usize = HEADER_LEN + FRAMES_PER_PACKET * FRAME_LEN;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub sequence: u32,
    pub first_frame: u32,
    pub frame_count: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    BadMagic,
    /// The payload length does not match `frame_count`.
    LengthMismatch,
}

impl Header {
    pub fn encode(&self) -> [u8; HEADER_LEN] {
        let mut out = [0; HEADER_LEN];
        out[0..4].copy_from_slice(&MAGIC);
        out[4..8].copy_from_slice(&self.sequence.to_le_bytes());
        out[8..12].copy_from_slice(&self.first_frame.to_le_bytes());
        out[12..14].copy_from_slice(&self.frame_count.to_le_bytes());
        out
    }
}

/// Splits a packet into its header and raw frames.
pub fn decode(packet: &[u8]) -> Result<(Header, &[[u8; FRAME_LEN]]), DecodeError> {
    let (header, payload) = packet
        .split_first_chunk::<HEADER_LEN>()
        .ok_or(DecodeError::TooShort)?;
    if header[0..4] != MAGIC {
        return Err(DecodeError::BadMagic);
    }

    // The ranges have fixed lengths, so the conversions to arrays cannot fail.
    let header = Header {
        sequence: u32::from_le_bytes(header[4..8].try_into().unwrap()),
        first_frame: u32::from_le_bytes(header[8..12].try_into().unwrap()),
        frame_count: u16::from_le_bytes(header[12..14].try_into().unwrap()),
    };

    let (frames, rest) = payload.as_chunks::<FRAME_LEN>();
    if !rest.is_empty() || frames.len() != usize::from(header.frame_count) {
        return Err(DecodeError::LengthMismatch);
    }
    Ok((header, frames))
}

/// Accumulates consecutive frames into one packet, without allocation.
pub struct PacketBuilder {
    buf: [u8; MAX_PACKET_LEN],
    first_frame: u32,
    count: usize,
}

impl PacketBuilder {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_PACKET_LEN],
            first_frame: 0,
            count: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn is_full(&self) -> bool {
        self.count == FRAMES_PER_PACKET
    }

    /// Whether the frame with conversion `index` can be appended: frames in
    /// a packet must be consecutive, so a gap means the packet has to be sent
    /// first.
    pub fn accepts(&self, index: u32) -> bool {
        self.is_empty()
            || (!self.is_full() && index == self.first_frame.wrapping_add(self.count as u32))
    }

    /// # Panics
    ///
    /// If [`accepts`](Self::accepts) is false for `index`.
    pub fn push(&mut self, index: u32, frame: &[u8; FRAME_LEN]) {
        assert!(self.accepts(index), "frame {index} does not fit the packet");
        if self.is_empty() {
            self.first_frame = index;
        }
        let offset = HEADER_LEN + self.count * FRAME_LEN;
        self.buf[offset..offset + FRAME_LEN].copy_from_slice(frame);
        self.count += 1;
    }

    /// Writes the header and returns the packet bytes. The builder is reset,
    /// the returned slice stays valid until the next `push`.
    pub fn finish(&mut self, sequence: u32) -> &[u8] {
        let header = Header {
            sequence,
            first_frame: self.first_frame,
            frame_count: self.count as u16,
        };
        self.buf[..HEADER_LEN].copy_from_slice(&header.encode());
        let len = HEADER_LEN + self.count * FRAME_LEN;
        self.count = 0;
        &self.buf[..len]
    }
}

impl Default for PacketBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    fn frame(tag: u8) -> [u8; FRAME_LEN] {
        let mut frame = [tag; FRAME_LEN];
        frame[0] = 0xC0;
        frame
    }

    #[test]
    fn packet_fits_mtu() {
        assert_eq!(MAX_PACKET_LEN, 1094);
        const { assert!(MAX_PACKET_LEN <= 1472) } // 1500 - IPv4 - UDP headers
    }

    #[test]
    fn round_trip() {
        let mut builder = PacketBuilder::new();
        for i in 0..3 {
            builder.push(100 + i, &frame(i as u8));
        }
        let packet = builder.finish(7);

        let (header, frames) = decode(packet).unwrap();
        assert_eq!(
            header,
            Header {
                sequence: 7,
                first_frame: 100,
                frame_count: 3
            }
        );
        assert_eq!(frames, [frame(0), frame(1), frame(2)]);
        assert!(builder.is_empty());
    }

    #[test]
    fn rejects_gaps_and_overflow() {
        let mut builder = PacketBuilder::new();
        builder.push(10, &frame(0));
        assert!(builder.accepts(11));
        assert!(!builder.accepts(12), "gap");
        assert!(!builder.accepts(10), "repeat");

        for i in 11..10 + FRAMES_PER_PACKET as u32 {
            builder.push(i, &frame(0));
        }
        assert!(builder.is_full());
        assert!(!builder.accepts(10 + FRAMES_PER_PACKET as u32));
    }

    #[test]
    fn index_wraps_around() {
        let mut builder = PacketBuilder::new();
        builder.push(u32::MAX, &frame(0));
        assert!(builder.accepts(0));
    }

    #[test]
    fn decode_errors() {
        let mut builder = PacketBuilder::new();
        builder.push(0, &frame(0));
        let packet = builder.finish(0).to_vec();

        assert_eq!(decode(&packet[..10]), Err(DecodeError::TooShort));
        let mut bad_magic = packet.clone();
        bad_magic[0] = b'X';
        assert_eq!(decode(&bad_magic), Err(DecodeError::BadMagic));
        assert_eq!(
            decode(&packet[..packet.len() - 1]),
            Err(DecodeError::LengthMismatch)
        );
    }
}
