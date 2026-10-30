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
use log::{error, info, warn};

use crate::board::AdsPins;
use crate::capture::TICKS_PER_US;
use crate::frontend::{AdsError, Conversion, Frontend};

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

/// Conversions never read: gaps in the hardware conversion count.
pub static MISSED: AtomicU32 = AtomicU32::new(0);

/// Longest time from `/DRDY` to the wake-up since the last
/// [`take_latency_peaks`], in microseconds.
static WAKE_PEAK_US: AtomicU32 = AtomicU32::new(0);

/// Longest time from `/DRDY` to the end of the frame read since the last
/// [`take_latency_peaks`], in microseconds. This is the one that must stay
/// below the sample period, so it shows how much margin the task has.
static READ_PEAK_US: AtomicU32 = AtomicU32::new(0);

/// Returns and resets the longest wake-up and read-done latencies, in
/// microseconds.
pub fn take_latency_peaks() -> (u32, u32) {
    (
        WAKE_PEAK_US.swap(0, Ordering::Relaxed),
        READ_PEAK_US.swap(0, Ordering::Relaxed),
    )
}

/// Frames whose read was still running when the next `/DRDY` came: they may
/// mix two conversions.
pub static OVERTAKEN: AtomicU32 = AtomicU32::new(0);

/// Frames lost to SPI errors or a corrupted status header.
pub static READ_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Resets, identifies, configures and starts the front-end.
async fn bring_up(frontend: &mut Frontend) -> Result<(), AdsError> {
    let ticks_per_us = frontend.measure_capture_ticks_per_us();
    if ticks_per_us == TICKS_PER_US {
        info!("Capture timer: {ticks_per_us} ticks/us");
    } else {
        warn!("Capture timer: {ticks_per_us} ticks/us, expected {TICKS_PER_US}: latency is off");
    }

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

    let mut expected: u32 = 0;

    loop {
        let Conversion {
            index,
            wake_us,
            read_us,
            overtaken,
            frame: result,
        } = frontend.next_frame().await;
        WAKE_PEAK_US.fetch_max(wake_us, Ordering::Relaxed);
        READ_PEAK_US.fetch_max(read_us, Ordering::Relaxed);
        if overtaken {
            OVERTAKEN.fetch_add(1, Ordering::Relaxed);
        }

        // Negative would mean the count missed the edge that woke us up; the
        // receiver then reports a duplicate, so it is not hidden here either.
        let gap = index.wrapping_sub(expected) as i32;
        if gap > 0 {
            MISSED.fetch_add(gap as u32, Ordering::Relaxed);
        }
        // A failed read still consumed a conversion.
        expected = index.wrapping_add(1);

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
    }
}
