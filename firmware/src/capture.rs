//! Hardware timestamps of `/DRDY` edges by the MCPWM capture module.
//!
//! The firmware learns about an edge only when it wakes up, so it cannot
//! tell by itself how late it is. The capture timer of MCPWM0 latches its
//! value on every `/DRDY` falling edge in hardware (channel 0); right after
//! the wake-up the firmware latches the same timer by software (channel 1).
//! The difference is the wake-up latency, exact to one timer tick.
//!
//! esp-hal does not implement the capture module yet, so this drives its
//! registers directly (ESP32-S3 TRM, chapter "Motor Control PWM"). esp-hal
//! still owns the peripheral: [`McPwm`] enables its clock and keeps it on.

use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputSignal};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;

/// The capture timer runs from APB_CLK (80 MHz), not from the PWM prescaler.
/// Checked at start-up by [`EdgeCapture::measure_ticks_per_us`].
pub const TICKS_PER_US: u32 = 80;

/// Capture channel latched by `/DRDY` falling edges.
const EDGE_CHANNEL: usize = 0;
/// Capture channel latched by software, to read the current time.
const NOW_CHANNEL: usize = 1;
/// `CAPn_MODE` bit 0: capture on the negative edge.
const MODE_NEGEDGE: u8 = 0b01;

type RegisterBlock = esp32s3::mcpwm0::RegisterBlock;

/// The last `/DRDY` edge and the current time, in timer ticks. The timer
/// wraps every 53 s; differences of `wrapping_sub` survive the wrap.
#[derive(Clone, Copy)]
pub struct Stamp {
    pub edge: u32,
    pub now: u32,
}

pub const fn ticks_to_us(ticks: u32) -> u32 {
    ticks / TICKS_PER_US
}

pub struct EdgeCapture {
    // Never read, but owning it keeps the MCPWM0 clock enabled.
    _mcpwm: McPwm<'static, MCPWM0<'static>>,
}

impl EdgeCapture {
    pub fn new(mcpwm: MCPWM0<'static>, drdy: &Input<'static>) -> Self {
        // Enables the bus clock and the register interface. The prescaler
        // only feeds the PWM timers, which stay unused.
        let mcpwm = McPwm::new(mcpwm, PeripheralClockConfig::with_prescaler(0));

        // Like PCNT, the capture input is one more listener of the same pin.
        InputSignal::PWM0_CAP0.connect_to(&drdy.peripheral_input());

        let regs = Self::regs();
        // Free-running: no sync input resets it.
        regs.cap_timer_cfg().write(|w| w.cap_timer_en().set_bit());
        regs.cap_ch_cfg(EDGE_CHANNEL).write(|w| {
            w.en().set_bit();
            // Prescale stays 0: every edge is captured.
            unsafe { w.mode().bits(MODE_NEGEDGE) }
        });
        // Mode 0, no edges: this channel is latched by software only.
        regs.cap_ch_cfg(NOW_CHANNEL).write(|w| w.en().set_bit());

        Self { _mcpwm: mcpwm }
    }

    fn regs() -> &'static RegisterBlock {
        // SAFETY: `_mcpwm` owns MCPWM0, so nothing else in the firmware uses
        // it, and esp-hal itself never touches the capture registers.
        unsafe { &*esp32s3::MCPWM0::PTR }
    }

    /// The current value of the capture timer.
    fn now() -> u32 {
        let regs = Self::regs();
        regs.cap_ch_cfg(NOW_CHANNEL).modify(|_, w| w.sw().set_bit());
        regs.cap_ch(NOW_CHANNEL).read().value().bits()
    }

    /// Takes the time of the last `/DRDY` falling edge and the current time.
    pub fn stamp(&self) -> Stamp {
        // Edge first: an edge between the two reads then makes the interval
        // longer, never negative.
        let edge = Self::regs().cap_ch(EDGE_CHANNEL).read().value().bits();
        Stamp {
            edge,
            now: Self::now(),
        }
    }

    /// Counts timer ticks over a busy-wait of 10 ms, to verify
    /// [`TICKS_PER_US`] on the real chip.
    pub fn measure_ticks_per_us(&self) -> u32 {
        const WAIT_US: u32 = 10_000;
        let start = Self::now();
        Delay::new().delay_micros(WAIT_US);
        Self::now().wrapping_sub(start) / WAIT_US
    }
}
