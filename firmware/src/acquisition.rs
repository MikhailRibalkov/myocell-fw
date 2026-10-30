//! Real-time sample acquisition from the ADS1298.
//!
//! [`acquisition_task`] owns the [`Frontend`] and appends every frame to the
//! store-and-forward ring in PSRAM ([`Backlog`]). It never waits for the
//! consumer: a full ring drops the frame and bumps [`OVERRUNS`], so data loss
//! is counted instead of hidden.
//!
//! The task must run on a high-priority interrupt executor: every frame has
//! to be read before the next `/DRDY`, whatever the thread mode is doing.

use core::sync::atomic::{AtomicU32, Ordering};

use ads1298::FRAME_LEN;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};
use log::{error, info};
use protocol::setup::DATA_RATE;

use crate::board::AdsPins;
use crate::frontend::{AdsError, Frontend};

/// Records in the ring: 12 MiB, about 3 minutes at 2 kSPS.
pub const BACKLOG_CAPACITY: usize = 12 * 1024 * 1024 / size_of::<Record>();

/// A raw frame tagged with its conversion index: 32 bytes with padding.
#[derive(Clone, Copy)]
pub struct Record {
    /// Counts every conversion since start, read or not: a gap between
    /// consecutive records is lost data.
    pub index: u32,
    pub raw: [u8; FRAME_LEN],
}

impl Record {
    pub const EMPTY: Self = Self {
        index: 0,
        raw: [0; FRAME_LEN],
    };
}

pub type Backlog = backlog::Backlog<'static, Record>;
pub type BacklogProducer = backlog::Producer<'static, 'static, Record>;
pub type BacklogConsumer = backlog::Consumer<'static, 'static, Record>;

/// Raised after every new record, so the sender need not poll the ring.
pub static NEW_DATA: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Records not acknowledged by the PC yet, published by the producer, which
/// keeps running even when the sender is blocked.
pub static UNACKED: AtomicU32 = AtomicU32::new(0);

/// Frames dropped because the ring was full of unacknowledged data.
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
pub async fn acquisition_task(pins: AdsPins, mut ring: BacklogProducer) {
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
            let periods = ((now - last) + period / 2).as_ticks() / period.as_ticks();
            // A late wake-up makes the next interval shorter than half a
            // period: that is 0 periods, and `0 - 1` must not wrap to u64::MAX.
            let missed = periods.saturating_sub(1);
            if missed > 0 {
                MISSED.fetch_add(missed as u32, Ordering::Relaxed);
                index = index.wrapping_add(missed as u32);
            }
        }

        match result {
            Ok(frame) => {
                let record = Record {
                    index,
                    raw: frame.raw,
                };
                if ring.push(record).is_err() {
                    OVERRUNS.fetch_add(1, Ordering::Relaxed);
                }
                NEW_DATA.signal(());
                UNACKED.store(ring.unacked(), Ordering::Relaxed);
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
