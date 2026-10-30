//! Real-time sample acquisition from the ADS1298.
//!
//! [`acquisition_task`] owns the [`Frontend`] and pushes every frame into
//! [`SAMPLES`]. It never waits for the consumer: a full queue drops the frame
//! and bumps [`OVERRUNS`], so data loss is counted instead of hidden.
//!
//! The task must run on a high-priority interrupt executor: every frame has
//! to be read before the next `/DRDY`, whatever the thread mode is doing.

use core::sync::atomic::{AtomicU32, Ordering};

use ads1298::FRAME_LEN;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant};
use log::{error, info};
use protocol::setup::DATA_RATE;

use crate::board::AdsPins;
use crate::frontend::{AdsError, Frontend};

/// 1 s of data at 2 kSPS (64 KiB): Wi-Fi occasionally stalls the consumer
/// for about a second (measured), and the queue has to ride that out.
pub const QUEUE_DEPTH: usize = 2048;

/// A raw frame tagged with its conversion index. Kept compact (32 bytes)
/// because the queue is deep; the PC parses the samples.
pub struct Sample {
    /// Counts every conversion since start, read or not: a gap between
    /// consecutive samples is lost data.
    pub index: u32,
    pub raw: [u8; FRAME_LEN],
}

pub static SAMPLES: Channel<CriticalSectionRawMutex, Sample, QUEUE_DEPTH> = Channel::new();

/// Frames dropped because [`SAMPLES`] was full.
pub static OVERRUNS: AtomicU32 = AtomicU32::new(0);

/// Conversions never read: detected as gaps between consecutive frames.
pub static MISSED: AtomicU32 = AtomicU32::new(0);

/// Frames lost to SPI errors or a corrupted status header.
pub static READ_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Resets, identifies, configures and starts the front-end.
async fn bring_up(frontend: &mut Frontend) -> Result<(), AdsError> {
    let model = frontend.power_up().await?;
    info!("ADS: {model:?}, {} channels", model.channels());
    frontend.configure().await?;
    info!("ADS registers: {:02x?}", frontend.dump_registers().await?);
    frontend.start().await
}

/// Takes raw pins rather than a ready [`Frontend`]: async esp-hal drivers are
/// `!Send`, so they are created on the executor that polls them.
#[embassy_executor::task]
pub async fn acquisition_task(pins: AdsPins) {
    let mut frontend = Frontend::new(pins);
    if let Err(e) = bring_up(&mut frontend).await {
        error!("ADS bring-up failed: {e:?}");
        return;
    }

    let period = Duration::from_hz(DATA_RATE.hz().into());
    let mut last_frame: Option<Instant> = None;
    let mut index: u32 = 0;

    loop {
        let result = frontend.next_frame().await;

        let now = Instant::now();
        if let Some(last) = last_frame.replace(now) {
            // Half a period of tolerance for wake-up jitter.
            let missed = ((now - last) + period / 2).as_ticks() / period.as_ticks() - 1;
            if missed > 0 {
                MISSED.fetch_add(missed as u32, Ordering::Relaxed);
                index = index.wrapping_add(missed as u32);
            }
        }

        match result {
            Ok(frame) => {
                let sample = Sample {
                    index,
                    raw: frame.raw,
                };
                if SAMPLES.try_send(sample).is_err() {
                    OVERRUNS.fetch_add(1, Ordering::Relaxed);
                }
            }
            Err(e) => {
                // Log only the first one: at 1 kSPS a persistent fault would
                // flood the console. The counter shows the rest.
                if READ_ERRORS.fetch_add(1, Ordering::Relaxed) == 0 {
                    error!("ADS frame read failed: {e:?}");
                }
            }
        }
        // A failed read still consumed a conversion.
        index = index.wrapping_add(1);
    }
}
