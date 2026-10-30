#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::sync::atomic::Ordering;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::Priority;
use esp_hal::psram::{Psram, PsramConfig, PsramMode};
use esp_hal::timer::timg::TimerGroup;
use esp_rtos::embassy::InterruptExecutor;
use log::info;
use myocell_fw::acquisition::{MISSED, OVERRUNS, QUEUE_DEPTH, READ_ERRORS, acquisition_task};
use myocell_fw::board::Board;
use myocell_fw::network::{self, DISCONNECTS, RSSI_DBM, STALL_RECONNECTS};
use myocell_fw::stream::{PACKETS_SENT, SEND_ERRORS, stream_task, take_peaks};

use static_cell::StaticCell;

extern crate alloc;

/// Runs real-time tasks: preempts everything in thread mode.
static REALTIME_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32s3 -o esp32s3-wroom-2 -o unstable-hal -o embassy -o alloc -o esp-backtrace -o log -o neovim-rustaceanvim

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let board = Board::new(peripherals);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // Wi-Fi allocates a lot and needs internal RAM (atomics do not work in
    // PSRAM), so give it a second internal region before PSRAM is added.
    esp_alloc::heap_allocator!(size: 64 * 1024);

    // The allocator is first-fit over regions in the order they were added,
    // so PSRAM goes second: small allocations stay in faster internal RAM.
    let psram = Psram::new(
        board.psram,
        PsramConfig {
            mode: PsramMode::OctalSpi,
            ..Default::default()
        },
    );
    let (psram_start, psram_size) = psram.raw_parts();
    info!("PSRAM: {} KiB at {:p}", psram_size / 1024, psram_start);
    esp_alloc::psram_allocator!(&psram);
    info!("{}", esp_alloc::HEAP.stats());

    // Too large for the internal heap, so it must land in PSRAM.
    let big = alloc::vec![0u8; 1024 * 1024];
    info!("1 MiB test buffer at {:p}", big.as_ptr());
    drop(big);

    let timg0 = TimerGroup::new(board.timg0);
    esp_rtos::start(timg0.timer0, board.sw_int);

    info!("Embassy initialized!");

    let realtime = REALTIME_EXECUTOR
        .init(InterruptExecutor::new(board.realtime_int))
        .start(Priority::Priority2);
    realtime.spawn(acquisition_task(board.ads).expect("task pool is free"));

    let stack = network::start(spawner, board.wifi);
    spawner.spawn(stream_task(stack).expect("task pool is free"));

    report_status().await
}

/// Logs device-side health once a second; the data itself goes to the PC.
async fn report_status() -> ! {
    let mut last_packets = 0;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        let packets = PACKETS_SENT.load(Ordering::Relaxed);
        let (send_peak_us, queue_peak) = take_peaks();
        info!(
            "tx {} pkt/s, send max {}.{} ms, queue peak {}/{} | rssi {} dBm, disconnects {}, stall reconnects {} | \
             send errors {}, missed {}, overruns {}, read errors {}",
            packets.wrapping_sub(last_packets),
            send_peak_us / 1000,
            send_peak_us % 1000 / 100,
            queue_peak,
            QUEUE_DEPTH,
            RSSI_DBM.load(Ordering::Relaxed),
            DISCONNECTS.load(Ordering::Relaxed),
            STALL_RECONNECTS.load(Ordering::Relaxed),
            SEND_ERRORS.load(Ordering::Relaxed),
            MISSED.load(Ordering::Relaxed),
            OVERRUNS.load(Ordering::Relaxed),
            READ_ERRORS.load(Ordering::Relaxed),
        );
        last_packets = packets;
    }
}
