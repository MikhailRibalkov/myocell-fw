//! Live samples for the web UI: what is sent and how it is batched.

use std::time::{Duration, Instant};

use protocol::setup::{CHANNEL_CONFIG, DATA_RATE};
use serde::Serialize;

use crate::stats::{ChannelStats, LinkStats};

/// How long samples are collected before they go out: short enough for a
/// smooth plot, long enough to keep the message count low.
pub const BATCH_PERIOD: Duration = Duration::from_millis(50);

/// Zero-based indices of the channels that are powered up.
pub fn enabled_channels() -> impl Iterator<Item = usize> {
    CHANNEL_CONFIG
        .iter()
        .enumerate()
        .filter(|(_, c)| !c.power_down)
        .map(|(i, _)| i)
}

/// A message to the browser, sent as JSON tagged by `type`.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Update {
    /// First message of every connection: how to read the ones that follow.
    Hello {
        sample_rate: u32,
        /// Names in the order of `Samples::channels`.
        channels: Vec<String>,
    },
    /// Consecutive conversions, from `first_index` on, in microvolts.
    Samples {
        first_index: u32,
        channels: Vec<Vec<f32>>,
    },
    /// A mark at conversion `index`, for the plots.
    Mark { index: u32, label: String },
    /// Statistics of the last second; `channels` is empty when no frame
    /// arrived.
    Stats {
        link: LinkStats,
        channels: Vec<ChannelStats>,
    },
    /// Sent on every change and once a second, so a new tab learns it too.
    Recording {
        /// The file being written; `None` when not recording.
        file: Option<String>,
        /// Recorded signal so far, by the conversion count.
        seconds: f64,
        marks: u32,
        /// Why the last recording ended on its own.
        error: Option<String>,
    },
}

impl Update {
    pub fn hello() -> Self {
        Self::Hello {
            sample_rate: DATA_RATE.hz(),
            channels: enabled_channels().map(|i| format!("CH{}", i + 1)).collect(),
        }
    }
}

/// Collects samples into [`Update::Samples`] batches. A batch is always
/// gapless: a lost or bad frame ends it, so the plot can show the gap
/// instead of joining the signal across it.
pub struct Batcher {
    first_index: u32,
    channels: Vec<Vec<f32>>,
    started: Instant,
}

impl Batcher {
    pub fn new(channel_count: usize) -> Self {
        Self {
            first_index: 0,
            channels: vec![Vec::new(); channel_count],
            started: Instant::now(),
        }
    }

    fn len(&self) -> usize {
        self.channels.first().map_or(0, Vec::len)
    }

    /// Adds the samples of conversion `index`. Returns the finished batch if
    /// `index` does not follow the previous one.
    pub fn push(&mut self, index: u32, samples: impl IntoIterator<Item = f32>) -> Option<Update> {
        let follows = self.first_index.wrapping_add(self.len() as u32) == index;
        let finished = if follows { None } else { self.take() };
        if self.len() == 0 {
            self.first_index = index;
            self.started = Instant::now();
        }
        for (channel, sample) in self.channels.iter_mut().zip(samples) {
            channel.push(sample);
        }
        finished
    }

    /// Returns the batch if it has been collecting for [`BATCH_PERIOD`].
    pub fn take_due(&mut self) -> Option<Update> {
        if self.started.elapsed() >= BATCH_PERIOD {
            self.take()
        } else {
            None
        }
    }

    fn take(&mut self) -> Option<Update> {
        if self.len() == 0 {
            return None;
        }
        let channels = self.channels.iter_mut().map(std::mem::take).collect();
        Some(Update::Samples {
            first_index: self.first_index,
            channels,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(first_index: u32, channels: &[&[f32]]) -> Update {
        Update::Samples {
            first_index,
            channels: channels.iter().map(|c| c.to_vec()).collect(),
        }
    }

    #[test]
    fn collects_consecutive_frames() {
        let mut batcher = Batcher::new(2);
        assert_eq!(batcher.push(10, [1.0, -1.0]), None);
        assert_eq!(batcher.push(11, [2.0, -2.0]), None);
        assert_eq!(
            batcher.take(),
            Some(samples(10, &[&[1.0, 2.0], &[-1.0, -2.0]]))
        );
        assert_eq!(batcher.take(), None);
    }

    #[test]
    fn splits_at_a_gap() {
        let mut batcher = Batcher::new(1);
        batcher.push(10, [1.0]);
        batcher.push(11, [2.0]);
        // 12 was lost: the batch before the gap comes out as it was.
        assert_eq!(batcher.push(13, [4.0]), Some(samples(10, &[&[1.0, 2.0]])));
        assert_eq!(batcher.take(), Some(samples(13, &[&[4.0]])));
    }

    #[test]
    fn continues_across_index_wrap() {
        let mut batcher = Batcher::new(1);
        batcher.push(u32::MAX, [1.0]);
        assert_eq!(batcher.push(0, [2.0]), None);
        assert_eq!(batcher.take(), Some(samples(u32::MAX, &[&[1.0, 2.0]])));
    }

    #[test]
    fn serializes_tagged() {
        let json = serde_json::to_string(&samples(7, &[&[1.5]])).unwrap();
        assert_eq!(
            json,
            r#"{"type":"samples","first_index":7,"channels":[[1.5]]}"#
        );
    }
}
