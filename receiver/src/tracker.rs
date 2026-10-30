//! Loss and duplicate accounting from packet headers.

use protocol::Header;

#[derive(Debug, Default)]
pub struct Tracker {
    next_sequence: Option<u32>,
    next_frame: Option<u32>,
    /// Conversions missing from the frame indices: lost on the device
    /// (TCP itself does not lose data).
    pub lost_frames: u64,
    /// Frames received twice: resent after a reconnect, before the device
    /// learned that they had arrived.
    pub duplicate_frames: u64,
    pub restarts: u64,
}

impl Tracker {
    /// Accounts for a packet and returns how many of its leading frames were
    /// already received and must be skipped.
    pub fn on_packet(&mut self, header: &Header) -> usize {
        // The device keeps counting packets across reconnects, so only a
        // restart makes the sequence go back.
        if let Some(expected) = self.next_sequence {
            // Wrapping difference as signed: correct across u32 overflow.
            if (header.sequence.wrapping_sub(expected) as i32) < 0 {
                self.restarts += 1;
                self.next_frame = None;
            }
        }
        self.next_sequence = Some(header.sequence.wrapping_add(1));

        let count = u32::from(header.frame_count);
        let end = header.first_frame.wrapping_add(count);
        let Some(next) = self.next_frame else {
            self.next_frame = Some(end);
            return 0;
        };

        let gap = header.first_frame.wrapping_sub(next) as i32;
        if gap >= 0 {
            self.lost_frames += gap as u64;
            self.next_frame = Some(end);
            0
        } else {
            let seen = gap.unsigned_abs().min(count);
            self.duplicate_frames += u64::from(seen);
            if seen < count {
                self.next_frame = Some(end);
            }
            seen as usize
        }
    }

    /// The first conversion index not received yet: what to acknowledge.
    pub fn next_frame(&self) -> Option<u32> {
        self.next_frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(sequence: u32, first_frame: u32, frame_count: u16) -> Header {
        Header {
            sequence,
            first_frame,
            frame_count,
        }
    }

    #[test]
    fn in_order_stream() {
        let mut tracker = Tracker::default();
        for i in 0..10 {
            assert_eq!(tracker.on_packet(&header(i, i * 40, 40)), 0);
        }
        assert_eq!(tracker.next_frame(), Some(400));
        assert_eq!((tracker.lost_frames, tracker.duplicate_frames), (0, 0));
    }

    #[test]
    fn device_side_gap_counts_lost_frames() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(0, 0, 7));
        tracker.on_packet(&header(1, 12, 40));
        assert_eq!(tracker.lost_frames, 5);
        assert_eq!(tracker.next_frame(), Some(52));
    }

    #[test]
    fn resent_frames_are_skipped() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(0, 0, 40));
        tracker.on_packet(&header(1, 40, 40));
        // Reconnect: the device had an ack up to 30 and resends from there,
        // with new sequence numbers.
        assert_eq!(tracker.on_packet(&header(2, 30, 40)), 40, "fully seen");
        assert_eq!(tracker.on_packet(&header(3, 70, 40)), 10, "partly seen");
        assert_eq!(tracker.on_packet(&header(4, 110, 40)), 0);
        assert_eq!(tracker.duplicate_frames, 50);
        assert_eq!(tracker.lost_frames, 0);
        assert_eq!(tracker.next_frame(), Some(150));
    }

    #[test]
    fn wraps_around() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(u32::MAX, u32::MAX - 9, 10));
        assert_eq!(tracker.on_packet(&header(0, 0, 40)), 0);
        assert_eq!((tracker.lost_frames, tracker.restarts), (0, 0));
    }

    #[test]
    fn device_restart_resyncs() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(500, 20_000, 40));
        assert_eq!(tracker.on_packet(&header(0, 3, 40)), 0);
        assert_eq!(tracker.restarts, 1);
        assert_eq!((tracker.lost_frames, tracker.duplicate_frames), (0, 0));
        assert_eq!(tracker.next_frame(), Some(43));
    }
}
