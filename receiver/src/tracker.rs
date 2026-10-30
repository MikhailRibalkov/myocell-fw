//! Loss accounting from packet headers.

use protocol::Header;

/// A packet this far behind is a device restart rather than a late packet.
const RESTART_THRESHOLD: i32 = 1000;

#[derive(Debug, Default)]
pub struct Tracker {
    next_sequence: Option<u32>,
    next_frame: Option<u32>,
    /// Packets missing from the sequence: lost in the network.
    pub lost_packets: u64,
    /// Conversions missing from the frame indices: lost anywhere.
    pub lost_frames: u64,
    /// Packets that arrived after a newer one and were dropped.
    pub late_packets: u64,
    pub restarts: u64,
}

impl Tracker {
    /// Accounts for a packet. Returns `false` for a late or duplicate packet,
    /// whose frames should be ignored.
    pub fn on_packet(&mut self, header: &Header) -> bool {
        if let Some(expected) = self.next_sequence {
            // Wrapping difference reinterpreted as signed: correct across the
            // u32 overflow as long as the real distance is below 2^31.
            let gap = header.sequence.wrapping_sub(expected) as i32;
            if gap < -RESTART_THRESHOLD {
                self.restarts += 1;
                self.next_frame = None;
            } else if gap < 0 {
                self.late_packets += 1;
                return false;
            } else {
                self.lost_packets += gap as u64;
            }
        }
        self.next_sequence = Some(header.sequence.wrapping_add(1));

        if let Some(expected) = self.next_frame {
            let gap = header.first_frame.wrapping_sub(expected) as i32;
            if gap > 0 {
                self.lost_frames += gap as u64;
            }
        }
        self.next_frame = Some(
            header
                .first_frame
                .wrapping_add(u32::from(header.frame_count)),
        );
        true
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
    fn in_order_stream_has_no_losses() {
        let mut tracker = Tracker::default();
        for i in 0..10 {
            assert!(tracker.on_packet(&header(i, i * 20, 20)));
        }
        assert_eq!((tracker.lost_packets, tracker.lost_frames), (0, 0));
    }

    #[test]
    fn lost_packet_counts_packet_and_frames() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(0, 0, 20));
        tracker.on_packet(&header(2, 40, 20));
        assert_eq!((tracker.lost_packets, tracker.lost_frames), (1, 20));
    }

    #[test]
    fn device_side_gap_counts_frames_only() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(0, 0, 7)); // flushed early because of the gap
        tracker.on_packet(&header(1, 12, 20));
        assert_eq!((tracker.lost_packets, tracker.lost_frames), (0, 5));
    }

    #[test]
    fn late_packet_is_dropped() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(5, 100, 20));
        assert!(!tracker.on_packet(&header(4, 80, 20)));
        assert_eq!(tracker.late_packets, 1);
    }

    #[test]
    fn wraps_around() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(u32::MAX, u32::MAX - 9, 10));
        assert!(tracker.on_packet(&header(0, 0, 20)));
        assert_eq!((tracker.lost_packets, tracker.lost_frames), (0, 0));
    }

    #[test]
    fn device_restart_resyncs() {
        let mut tracker = Tracker::default();
        tracker.on_packet(&header(50_000, 1_000_000, 20));
        assert!(tracker.on_packet(&header(0, 0, 20)));
        assert!(tracker.on_packet(&header(1, 20, 20)));
        assert_eq!(tracker.restarts, 1);
        assert_eq!((tracker.lost_packets, tracker.lost_frames), (0, 0));
    }
}
