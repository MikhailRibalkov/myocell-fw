//! Pin assignment of the myocell_s3 board (rev 2026.r1).
//!
//! Source of truth: KiCad project `RF-Lab/emg-lab/myocell_s3`
//! (nets checked against `myocell_s3.kicad_pcb`).
//!
//! The board consumes [`Peripherals`] as a whole, so every peripheral that is
//! not listed here is dropped and cannot be used by accident. In particular:
//! - GPIO33..GPIO37 are used by the octal flash/PSRAM of the WROOM-2 module;
//! - GPIO1 is routed to LED D8, which is not populated;
//! - GPIO3 and GPIO46 are strapping pins pulled down on the board.

use esp_hal::peripherals::{
    ADC1, FROM_CPU_INTR0, FROM_CPU_INTR1, GPIO0, GPIO7, GPIO8, GPIO10, GPIO11, GPIO12, GPIO13,
    GPIO14, GPIO21, GPIO47, GPIO48, MCPWM0, PCNT, PSRAM, Peripherals, SPI2, TIMG0, WIFI,
};

/// Signals between the ESP32-S3 and the ADS1298 analog front-end.
pub struct AdsPins {
    /// GP-SPI2 (FSPI): GPIO10..13 are its native IO MUX pins.
    pub spi: SPI2<'static>,
    pub sclk: GPIO12<'static>,
    /// ESP MOSI -> ADS `DIN`.
    pub mosi: GPIO11<'static>,
    /// ADS `DOUT` -> ESP MISO.
    pub miso: GPIO13<'static>,
    /// ADS `/CS`, active low.
    pub cs: GPIO10<'static>,
    /// ADS `/DRDY`, active low: a new sample is ready.
    pub drdy: GPIO14<'static>,
    /// Counts `/DRDY` falling edges in hardware, so every conversion is
    /// numbered even when the firmware wakes up too late to read it.
    pub pcnt: PCNT<'static>,
    /// Timestamps `/DRDY` edges in hardware with its capture module, to
    /// measure how late the firmware wakes up.
    pub mcpwm: MCPWM0<'static>,
    /// ADS `START`, active high: starts conversions.
    pub start: GPIO21<'static>,
    /// ADS `/RESET`, active low.
    ///
    /// GPIO47/48 belong to the VDD_SPI power domain, which is 1.8 V on the
    /// WROOM-2. A 1.8 V high level is below the ADS1298 VIH (0.8 * DVDD),
    /// so prefer the SPI `RESET` command over toggling this pin.
    pub reset: GPIO48<'static>,
    /// ADS `/PWDN`, active low.
    pub pwdn: GPIO8<'static>,
    /// ADS `CLK`. `CLKSEL` is tied high, so the ADS runs from its internal
    /// oscillator. Keep this pin as an input and never set `CONFIG1.CLK_EN`:
    /// the ADS would drive 3.3 V into a 1.8 V pin (see `reset`).
    pub clk: GPIO47<'static>,
}

/// All board resources used by the firmware.
pub struct Board {
    pub ads: AdsPins,
    /// BOOT button SW2, shorts to GND when pressed (needs a pull-up).
    pub boot_button: GPIO0<'static>,
    /// Battery voltage divider, ADC1 channel 6 (usable together with Wi-Fi).
    pub bat_mon: GPIO7<'static>,
    pub adc1: ADC1<'static>,
    /// Octal PSRAM of the WROOM-2 module (8 or 16 MB).
    pub psram: PSRAM<'static>,
    /// Wi-Fi radio of the ESP32-S3.
    pub wifi: WIFI<'static>,
    /// Timer group used as the time driver of `esp-rtos`/Embassy.
    pub timg0: TIMG0<'static>,
    /// Software interrupt used by `esp-rtos` to wake the executor.
    pub sw_int: FROM_CPU_INTR0<'static>,
    /// Software interrupt driving the high-priority executor for real-time tasks.
    pub realtime_int: FROM_CPU_INTR1<'static>,
}

impl Board {
    pub fn new(p: Peripherals) -> Self {
        Self {
            ads: AdsPins {
                spi: p.SPI2,
                sclk: p.GPIO12,
                mosi: p.GPIO11,
                miso: p.GPIO13,
                cs: p.GPIO10,
                drdy: p.GPIO14,
                pcnt: p.PCNT,
                mcpwm: p.MCPWM0,
                start: p.GPIO21,
                reset: p.GPIO48,
                pwdn: p.GPIO8,
                clk: p.GPIO47,
            },
            boot_button: p.GPIO0,
            bat_mon: p.GPIO7,
            adc1: p.ADC1,
            psram: p.PSRAM,
            wifi: p.WIFI,
            timg0: p.TIMG0,
            sw_int: p.FROM_CPU_INTR0,
            realtime_int: p.FROM_CPU_INTR1,
        }
    }
}
