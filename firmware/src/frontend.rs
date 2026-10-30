//! ADS1298 analog front-end wired to the ESP32-S3 (see [`AdsPins`]).
//!
//! This is the board-specific glue: it turns raw pins into esp-hal drivers,
//! builds an [`SpiDevice`](embedded_hal_async::spi::SpiDevice) for the
//! [`ads1298`] driver and runs the power-up sequence.

use core::convert::Infallible;

use ads1298::{Ads1298, CONFIG2_DEFAULT, Command, Frame, Model, Register};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use esp_hal::Async;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use protocol::setup::{CHANNEL_CONFIG, CONFIG3, DATA_RATE, RLD_SENSE};

use crate::board::AdsPins;

/// A frame takes ~30 us at 8 MHz, well inside the 1 ms sample period.
/// The ADS1298 accepts up to 20 MHz.
const SPI_FREQUENCY: Rate = Rate::from_mhz(8);

/// Power-on reset and oscillator start-up: 2^18 t_CLK = 128 ms, plus margin.
const POWER_UP_DELAY: Duration = Duration::from_millis(150);

/// Reset execution time after the `RESET` command: 18 t_CLK = 8.8 us.
const RESET_DELAY: Duration = Duration::from_micros(10);

/// Settling time of the internal reference buffer after it is enabled.
const REFERENCE_SETTLE_DELAY: Duration = Duration::from_millis(150);

/// The SPI bus plus software-driven /CS, as seen by the driver.
pub type AdsSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
pub type AdsError = ads1298::Error<DeviceError<esp_hal::spi::Error, Infallible>>;

pub struct Frontend {
    ads: Ads1298<AdsSpi>,
    start: Output<'static>,
    drdy: Input<'static>,
    // Never read, but owning the drivers keeps the levels set in `new`.
    _pwdn: Output<'static>,
    _reset: Output<'static>,
    _clk: Input<'static>,
}

impl Frontend {
    /// Configures the pins and the SPI bus. Does not talk to the device yet.
    pub fn new(pins: AdsPins) -> Self {
        let spi_config = SpiConfig::default()
            .with_frequency(SPI_FREQUENCY)
            .with_mode(Mode::_1);
        let bus = Spi::new(pins.spi, spi_config)
            .expect("valid SPI config")
            .with_sck(pins.sclk)
            .with_mosi(pins.mosi)
            .with_miso(pins.miso)
            .into_async();
        let cs = Output::new(pins.cs, Level::High, OutputConfig::default());
        let spi = ExclusiveDevice::new(bus, cs, Delay).unwrap_or_else(|e| match e {});

        Self {
            ads: Ads1298::new(spi),
            // Conversions are started later, after configuration.
            start: Output::new(pins.start, Level::Low, OutputConfig::default()),
            drdy: Input::new(pins.drdy, InputConfig::default()),
            _pwdn: Output::new(pins.pwdn, Level::High, OutputConfig::default()),
            // 1.8 V high level, see `AdsPins::reset`. Reset is done over SPI.
            _reset: Output::new(pins.reset, Level::High, OutputConfig::default()),
            // High impedance: the ADS runs from its internal oscillator.
            _clk: Input::new(pins.clk, InputConfig::default()),
        }
    }

    /// Resets the device, leaves it in SDATAC mode and returns its model.
    pub async fn power_up(&mut self) -> Result<Model, AdsError> {
        Timer::after(POWER_UP_DELAY).await;

        self.ads.command(Command::Reset).await?;
        Timer::after(RESET_DELAY).await;
        // The device wakes up in RDATAC mode and ignores register commands.
        self.ads.command(Command::Sdatac).await?;

        self.ads.read_model().await
    }

    /// Reads the whole register map, `ID` through `WCT2`.
    pub async fn dump_registers(&mut self) -> Result<[u8; 26], AdsError> {
        let mut regs = [0; 26];
        self.ads.read_registers(Register::Id, &mut regs).await?;
        Ok(regs)
    }

    /// Writes the configuration from [`protocol::setup`]. The device must be
    /// in SDATAC mode.
    pub async fn configure(&mut self) -> Result<(), AdsError> {
        self.ads
            .write_register(Register::Config3, CONFIG3.to_register())
            .await?;
        Timer::after(REFERENCE_SETTLE_DELAY).await;

        self.ads
            .write_register(Register::Config1, DATA_RATE.config1())
            .await?;
        self.ads
            .write_register(Register::Config2, CONFIG2_DEFAULT)
            .await?;
        for (channel, config) in (1..).zip(CHANNEL_CONFIG) {
            self.ads.configure_channel(channel, config).await?;
        }
        self.ads
            .write_register(Register::RldSensP, RLD_SENSE)
            .await?;
        self.ads
            .write_register(Register::RldSensN, RLD_SENSE)
            .await?;
        Ok(())
    }

    /// Starts continuous conversions: one frame per `/DRDY` falling edge.
    pub async fn start(&mut self) -> Result<(), AdsError> {
        // RDATAC first, so the very first conversion is already streamed.
        self.ads.command(Command::Rdatac).await?;
        self.start.set_high();
        Ok(())
    }

    /// Waits for the next conversion and reads it.
    pub async fn next_frame(&mut self) -> Result<Frame, AdsError> {
        self.drdy.wait_for_falling_edge().await;
        self.ads.read_frame().await
    }
}
