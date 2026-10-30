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
use esp_hal::pcnt::Pcnt;
use esp_hal::pcnt::channel::{CtrlMode, EdgeMode};
use esp_hal::pcnt::unit::Unit;
use esp_hal::peripherals::PCNT;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use protocol::setup::{CHANNEL_CONFIG, CONFIG3, DATA_RATE, RLD_SENSE};

use crate::board::AdsPins;
use crate::capture::{EdgeCapture, ticks_to_us};

/// A frame takes ~30 us at 8 MHz, well inside the 1 ms sample period.
/// The ADS1298 accepts up to 20 MHz.
const SPI_FREQUENCY: Rate = Rate::from_mhz(8);

/// Power-on reset and oscillator start-up: 2^18 t_CLK = 128 ms, plus margin.
const POWER_UP_DELAY: Duration = Duration::from_millis(150);

/// Reset execution time after the `RESET` command: 18 t_CLK = 8.8 us.
const RESET_DELAY: Duration = Duration::from_micros(10);

/// Settling time of the internal reference buffer after it is enabled.
const REFERENCE_SETTLE_DELAY: Duration = Duration::from_millis(150);

/// Glitch filter of the `/DRDY` counter in APB cycles (80 MHz): 10 = 125 ns.
/// An unread conversion leaves `/DRDY` high for only 4 t_CLK (~2 us) before
/// the next falling edge, so the filter must stay far below that. It must
/// also be shorter than the wake-up latency, or the count read right after
/// the wake-up would not include the edge yet.
const DRDY_FILTER_CYCLES: u16 = 10;

/// The SPI bus plus software-driven /CS, as seen by the driver.
pub type AdsSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
pub type AdsError = ads1298::Error<DeviceError<esp_hal::spi::Error, Infallible>>;

/// Counts conversions by `/DRDY` falling edges, independently of software.
struct ConversionCounter {
    unit: Unit<'static, 0>,
    /// Hardware count at the previous `read`.
    last: u16,
    /// Conversions since the last `reset`, extended to 32 bits.
    total: u32,
}

impl ConversionCounter {
    fn new(pcnt: PCNT<'static>, drdy: &Input<'static>) -> Self {
        let unit = Pcnt::new(pcnt).unit0;
        // No limits: the 16-bit count wraps and `read` extends it.
        unit.set_low_limit(None).unwrap_or_else(|_| unreachable!());
        unit.set_high_limit(None).unwrap_or_else(|_| unreachable!());
        unit.set_filter(Some(DRDY_FILTER_CYCLES))
            .expect("filter threshold is at most 1023");

        // The GPIO matrix routes the pin to PCNT as well, so the same pin
        // still drives the `/DRDY` interrupt.
        unit.channel0.set_edge_signal(drdy.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);
        unit.channel0.set_ctrl_mode(CtrlMode::Keep, CtrlMode::Keep);
        unit.channel1.set_input_mode(EdgeMode::Hold, EdgeMode::Hold);

        unit.clear();
        unit.resume();
        Self {
            unit,
            last: 0,
            total: 0,
        }
    }

    fn reset(&mut self) {
        self.unit.clear();
        self.last = 0;
        self.total = 0;
    }

    /// Conversions since the last `reset`. Must be called at least once per
    /// 65535 conversions (32 s at 2 kSPS), or the extension loses a wrap.
    fn read(&mut self) -> u32 {
        // The bits are the same: as `u16` the count simply wraps at 65536.
        let count = self.unit.value() as u16;
        self.total = self
            .total
            .wrapping_add(u32::from(count.wrapping_sub(self.last)));
        self.last = count;
        self.total
    }
}

/// One conversion as returned by [`Frontend::next_frame`].
pub struct Conversion {
    /// Counted in hardware since [`Frontend::start`].
    pub index: u32,
    /// From the `/DRDY` edge to the wake-up, measured in hardware.
    pub wake_us: u32,
    /// From the `/DRDY` edge to the end of the read: must stay below the
    /// sample period, when the ADS overwrites its output register.
    pub read_us: u32,
    /// The next `/DRDY` came before the read finished: the frame may mix
    /// two conversions.
    pub overtaken: bool,
    pub frame: Result<Frame, AdsError>,
}

pub struct Frontend {
    ads: Ads1298<AdsSpi>,
    start: Output<'static>,
    drdy: Input<'static>,
    conversions: ConversionCounter,
    edges: EdgeCapture,
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
        let drdy = Input::new(pins.drdy, InputConfig::default());
        let conversions = ConversionCounter::new(pins.pcnt, &drdy);
        let edges = EdgeCapture::new(pins.mcpwm, &drdy);

        Self {
            ads: Ads1298::new(spi),
            // Conversions are started later, after configuration.
            start: Output::new(pins.start, Level::Low, OutputConfig::default()),
            drdy,
            conversions,
            edges,
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
        // The first conversion after this gets index 0.
        self.conversions.reset();
        self.start.set_high();
        Ok(())
    }

    /// Capture timer rate measured on the chip, to check [`TICKS_PER_US`].
    pub fn measure_capture_ticks_per_us(&self) -> u32 {
        self.edges.measure_ticks_per_us()
    }

    /// Waits for the next conversion and reads it.
    ///
    /// The ADS keeps only the latest conversion. After a wake-up so late
    /// that newer conversions have overwritten the awaited one, the frame
    /// read is the newest, and its index skips the lost ones.
    pub async fn next_frame(&mut self) -> Conversion {
        self.drdy.wait_for_falling_edge().await;
        let woken = self.edges.stamp();
        // Counted before the read: it names the conversion the output
        // register holds right now.
        let index = self.conversions.read().wrapping_sub(1);
        let frame = self.ads.read_frame().await;
        let done = self.edges.stamp();

        Conversion {
            index,
            wake_us: ticks_to_us(woken.now.wrapping_sub(woken.edge)),
            // From the edge taken at wake-up: by now a newer edge may have
            // replaced it, and measuring from that one would hide the delay.
            read_us: ticks_to_us(done.now.wrapping_sub(woken.edge)),
            overtaken: done.edge != woken.edge,
            frame,
        }
    }
}
