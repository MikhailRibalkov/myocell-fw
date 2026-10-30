//! Splits the TCP byte stream into packets.

use protocol::{DecodeError, FRAME_LEN, HEADER_LEN, Header};

/// A packet taken off the stream.
#[derive(Debug, PartialEq)]
pub struct Packet {
    pub header: Header,
    pub frames: Vec<[u8; FRAME_LEN]>,
}

/// TCP delivers bytes in arbitrary chunks: a packet may arrive in pieces,
/// or together with the start of the next one. Bytes are buffered until a
/// whole packet is there.
#[derive(Default)]
pub struct StreamParser {
    buf: Vec<u8>,
}

impl StreamParser {
    pub fn extend(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    /// The next complete packet, `None` if more bytes are needed.
    ///
    /// An error means the stream lost the packet boundaries; there is no
    /// way to find them again, so the connection has to be dropped.
    pub fn next_packet(&mut self) -> Option<Result<Packet, DecodeError>> {
        let header_bytes = self.buf.first_chunk::<HEADER_LEN>()?;
        let header = match protocol::decode_header(header_bytes) {
            Ok(header) => header,
            Err(e) => return Some(Err(e)),
        };
        let len = HEADER_LEN + header.payload_len();
        if self.buf.len() < len {
            return None;
        }
        let (frames, _) = self.buf[HEADER_LEN..len].as_chunks::<FRAME_LEN>();
        let frames = frames.to_vec();
        self.buf.drain(..len);
        Some(Ok(Packet { header, frames }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::PacketBuilder;

    fn packet(sequence: u32, first: u32, count: u32) -> Vec<u8> {
        let mut builder = PacketBuilder::new();
        for i in 0..count {
            builder.push(first + i, &[0xC0; FRAME_LEN]);
        }
        builder.finish(sequence).to_vec()
    }

    #[test]
    fn reassembles_split_packets() {
        let bytes = [packet(0, 0, 3), packet(1, 3, 2)].concat();
        let mut parser = StreamParser::default();

        // Feed in awkward pieces: mid-header, mid-frame, two packets at once.
        for chunk in bytes.chunks(10) {
            parser.extend(chunk);
        }
        let first = parser.next_packet().unwrap().unwrap();
        assert_eq!((first.header.sequence, first.frames.len()), (0, 3));
        let second = parser.next_packet().unwrap().unwrap();
        assert_eq!((second.header.sequence, second.frames.len()), (1, 2));
        assert!(parser.next_packet().is_none());
    }

    #[test]
    fn waits_for_the_rest() {
        let bytes = packet(0, 0, 3);
        let mut parser = StreamParser::default();
        parser.extend(&bytes[..bytes.len() - 1]);
        assert!(parser.next_packet().is_none());
        parser.extend(&bytes[bytes.len() - 1..]);
        assert!(parser.next_packet().is_some());
    }

    #[test]
    fn reports_lost_boundaries() {
        let mut parser = StreamParser::default();
        parser.extend(&[0xAA; HEADER_LEN]);
        assert_eq!(parser.next_packet(), Some(Err(DecodeError::BadMagic)));
    }
}
