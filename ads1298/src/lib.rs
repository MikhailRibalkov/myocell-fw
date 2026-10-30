//! Async driver for the TI ADS1294/6/8(R) analog front-end.
//!
//! The driver is generic over [`SpiDevice`], so it knows nothing about the MCU.
//! The SPI bus must be configured for mode 1 (CPOL = 0, CPHA = 1), MSB first,
//! SCLK up to 20 MHz.
//!
//! Timings assume the internal 2.048 MHz oscillator (`CLKSEL` pin high).

#![no_std]

use embedded_hal_async::spi::{Operation, SpiDevice};

/// Period of the 2.048 MHz master clock, rounded up.
const T_CLK_NS: u32 = 489;

/// Opcode decode time (t_SDECODE) and SCLK-to-/CS-high delay (t_SCCS): 4 t_CLK.
///
/// Inserted after every opcode byte, since the device needs it to decode a
/// command before the next byte arrives.
const T_DECODE_NS: u32 = 4 * T_CLK_NS;

/// First byte of the `RREG` opcode, OR-ed with the start register address.
const RREG: u8 = 0x20;
/// First byte of the `WREG` opcode, OR-ed with the start register address.
const WREG: u8 = 0x40;

/// Single-byte SPI commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    Wakeup = 0x02,
    Standby = 0x04,
    Reset = 0x06,
    Start = 0x08,
    Stop = 0x0A,
    /// Enable Read Data Continuous mode (the power-up default).
    Rdatac = 0x10,
    /// Stop Read Data Continuous mode. Required before `RREG`/`WREG`.
    Sdatac = 0x11,
    /// Read one sample in non-continuous mode.
    Rdata = 0x12,
}

/// Register map.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    Id = 0x00,
    Config1 = 0x01,
    Config2 = 0x02,
    Config3 = 0x03,
    Loff = 0x04,
    Ch1Set = 0x05,
    Ch2Set = 0x06,
    Ch3Set = 0x07,
    Ch4Set = 0x08,
    Ch5Set = 0x09,
    Ch6Set = 0x0A,
    Ch7Set = 0x0B,
    Ch8Set = 0x0C,
    RldSensP = 0x0D,
    RldSensN = 0x0E,
    LoffSensP = 0x0F,
    LoffSensN = 0x10,
    LoffFlip = 0x11,
    LoffStatP = 0x12,
    LoffStatN = 0x13,
    Gpio = 0x14,
    Pace = 0x15,
    Resp = 0x16,
    Config4 = 0x17,
    Wct1 = 0x18,
    Wct2 = 0x19,
}

/// Device model decoded from the `ID` register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    Ads1294,
    Ads1296,
    Ads1298,
    Ads1294R,
    Ads1296R,
    Ads1298R,
}

impl Model {
    /// Decodes the `ID` register: `DEV_ID[7:5]`, `0b10` in `[4:3]`, `NU_CH[2:0]`.
    pub fn from_id(id: u8) -> Option<Self> {
        let respiration = match id >> 5 {
            0b100 => false,
            0b110 => true,
            _ => return None,
        };
        if (id >> 3) & 0b11 != 0b10 {
            return None;
        }
        let model = match (id & 0b111, respiration) {
            (0b000, false) => Self::Ads1294,
            (0b001, false) => Self::Ads1296,
            (0b010, false) => Self::Ads1298,
            (0b000, true) => Self::Ads1294R,
            (0b001, true) => Self::Ads1296R,
            (0b010, true) => Self::Ads1298R,
            _ => return None,
        };
        Some(model)
    }

    pub fn channels(self) -> usize {
        match self {
            Self::Ads1294 | Self::Ads1294R => 4,
            Self::Ads1296 | Self::Ads1296R => 6,
            Self::Ads1298 | Self::Ads1298R => 8,
        }
    }
}

/// `CONFIG2`: internal test signal, 1 x (VREFP - VREFN) / 2.4 mV,
/// square wave at f_CLK / 2^21 (~1 Hz).
pub const CONFIG2_TEST_SIGNAL: u8 = 0x10;

/// `CONFIG2`: test signal off (reset value).
pub const CONFIG2_DEFAULT: u8 = 0x00;

/// Reference voltage with the internal reference buffer, see [`Config3`].
pub const VREF_VOLTS: f32 = 2.4;

/// Contents of `CONFIG3`. VREFP is fixed at 2.4 V ([`VREF_VOLTS`]);
/// RLD measurement and lead-off sensing are off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config3 {
    /// Internal reference buffer (`PD_REFBUF`).
    pub internal_reference: bool,
    /// RLD amplifier powered (`PD_RLD`).
    pub rld_buffer: bool,
    /// RLD reference (AVDD + AVSS) / 2 generated internally (`RLDREF_INT`)
    /// instead of taken from the `RLDREF` pin.
    pub rld_internal_reference: bool,
}

impl Config3 {
    pub const fn to_register(self) -> u8 {
        const RESERVED: u8 = 1 << 6; // must be written as 1
        (self.internal_reference as u8) << 7
            | RESERVED
            | (self.rld_internal_reference as u8) << 3
            | (self.rld_buffer as u8) << 2
    }
}

/// Output data rate in high-resolution mode (`CONFIG1.DR`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DataRate {
    Sps32k = 0b000,
    Sps16k = 0b001,
    Sps8k = 0b010,
    Sps4k = 0b011,
    Sps2k = 0b100,
    Sps1k = 0b101,
    Sps500 = 0b110,
}

impl DataRate {
    pub const fn hz(self) -> u32 {
        32_000 >> self as u32
    }

    /// `CONFIG1` value: high-resolution mode, daisy-chain mode,
    /// oscillator clock output (`CLK_EN`) disabled.
    pub const fn config1(self) -> u8 {
        0x80 | self as u8
    }
}

/// PGA gain (`CHnSET.GAIN`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gain {
    X1,
    X2,
    X3,
    X4,
    X6,
    X8,
    X12,
}

impl Gain {
    const fn bits(self) -> u8 {
        match self {
            Self::X6 => 0b000,
            Self::X1 => 0b001,
            Self::X2 => 0b010,
            Self::X3 => 0b011,
            Self::X4 => 0b100,
            Self::X8 => 0b101,
            Self::X12 => 0b110,
        }
    }

    pub const fn factor(self) -> u8 {
        match self {
            Self::X1 => 1,
            Self::X2 => 2,
            Self::X3 => 3,
            Self::X4 => 4,
            Self::X6 => 6,
            Self::X8 => 8,
            Self::X12 => 12,
        }
    }

    /// Converts a raw sample to microvolts at the channel input.
    pub fn to_microvolts(self, raw: i32, vref_volts: f32) -> f32 {
        const FULL_SCALE: f32 = 8_388_607.0; // 2^23 - 1
        raw as f32 * vref_volts * 1e6 / (f32::from(self.factor()) * FULL_SCALE)
    }
}

/// Channel input multiplexer (`CHnSET.MUXn`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ChannelInput {
    Normal = 0b000,
    Shorted = 0b001,
    RldMeasure = 0b010,
    Supply = 0b011,
    Temperature = 0b100,
    TestSignal = 0b101,
    RldDrp = 0b110,
    RldDrn = 0b111,
}

/// Contents of a `CHnSET` register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelConfig {
    pub power_down: bool,
    pub gain: Gain,
    pub input: ChannelInput,
}

impl ChannelConfig {
    pub const fn to_register(self) -> u8 {
        (self.power_down as u8) << 7 | self.gain.bits() << 4 | self.input as u8
    }
}

pub const CHANNELS: usize = 8;

/// Bytes clocked out per conversion in RDATAC mode: 24-bit status plus
/// one 24-bit sample per channel.
pub const FRAME_LEN: usize = 3 * (1 + CHANNELS);

/// One conversion result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// `1100` | `LOFF_STATP[7:0]` | `LOFF_STATN[7:0]` | `GPIO[4:1]`.
    pub status: u32,
    /// Two's complement samples, sign-extended to 32 bits.
    pub samples: [i32; CHANNELS],
    /// The frame exactly as clocked out, for lossless forwarding.
    pub raw: [u8; FRAME_LEN],
}

impl Frame {
    /// Parses a raw frame; `None` if the status header is not `1100`,
    /// which means the frame is misaligned or the bus is broken.
    pub fn parse(raw: &[u8; FRAME_LEN]) -> Option<Self> {
        let status = u32::from_be_bytes([0, raw[0], raw[1], raw[2]]);
        if status >> 20 != 0b1100 {
            return None;
        }

        let mut samples = [0; CHANNELS];
        let (chunks, _) = raw[3..].as_chunks::<3>();
        for (sample, &[high, mid, low]) in samples.iter_mut().zip(chunks) {
            // Put the 24 bits on top of an i32, then shift back: `>>` on a
            // signed integer is arithmetic and replicates the sign bit.
            *sample = i32::from_be_bytes([high, mid, low, 0]) >> 8;
        }
        Some(Self {
            status,
            samples,
            raw: *raw,
        })
    }

    /// Lead-off status of the positive inputs, bit n = channel n + 1.
    pub fn lead_off_p(&self) -> u8 {
        (self.status >> 12) as u8
    }

    /// Lead-off status of the negative inputs, bit n = channel n + 1.
    pub fn lead_off_n(&self) -> u8 {
        (self.status >> 4) as u8
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Spi(E),
    /// The `ID` register holds a value that matches no known model.
    /// `0x00` or `0xFF` usually means the device does not respond at all.
    UnknownId(u8),
    /// The frame does not start with the `1100` status header.
    /// Holds the first status byte.
    InvalidFrame(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

pub struct Ads1298<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Ads1298<SPI> {
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Gives the SPI device back.
    pub fn release(self) -> SPI {
        self.spi
    }

    pub async fn command(&mut self, command: Command) -> Result<(), SPI::Error> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[command as u8]),
                Operation::DelayNs(T_DECODE_NS),
            ])
            .await
    }

    /// Reads `buf.len()` consecutive registers starting at `start`.
    ///
    /// The device must be in SDATAC mode, see [`Command::Sdatac`].
    ///
    /// Uses one `RREG` per register: the device needs about 4 t_CLK per
    /// output byte, so a multi-register read repeats bytes at fast SCLK
    /// (observed at 8 MHz), and delays cannot be put between the bytes of
    /// a single `Operation::Read`.
    pub async fn read_registers(
        &mut self,
        start: Register,
        buf: &mut [u8],
    ) -> Result<(), SPI::Error> {
        for (address, value) in (start as u8..).zip(buf.iter_mut()) {
            self.spi
                .transaction(&mut [
                    Operation::Write(&[RREG | address]),
                    Operation::DelayNs(T_DECODE_NS),
                    Operation::Write(&[0]),
                    Operation::DelayNs(T_DECODE_NS),
                    Operation::Read(core::slice::from_mut(value)),
                    Operation::DelayNs(T_DECODE_NS),
                ])
                .await?;
        }
        Ok(())
    }

    pub async fn read_register(&mut self, register: Register) -> Result<u8, SPI::Error> {
        let mut buf = [0];
        self.read_registers(register, &mut buf).await?;
        Ok(buf[0])
    }

    /// Writes a single register. The device must be in SDATAC mode.
    pub async fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), SPI::Error> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[WREG | register as u8]),
                Operation::DelayNs(T_DECODE_NS),
                Operation::Write(&[0]),
                Operation::DelayNs(T_DECODE_NS),
                Operation::Write(&[value]),
                Operation::DelayNs(T_DECODE_NS),
            ])
            .await
    }

    /// Reads the `ID` register and decodes the device model.
    pub async fn read_model(&mut self) -> Result<Model, Error<SPI::Error>> {
        let id = self.read_register(Register::Id).await?;
        Model::from_id(id).ok_or(Error::UnknownId(id))
    }

    /// Writes a `CHnSET` register; `channel` is 1-based.
    ///
    /// # Panics
    ///
    /// If `channel` is not in `1..=8`.
    pub async fn configure_channel(
        &mut self,
        channel: u8,
        config: ChannelConfig,
    ) -> Result<(), SPI::Error> {
        let register = match channel {
            1 => Register::Ch1Set,
            2 => Register::Ch2Set,
            3 => Register::Ch3Set,
            4 => Register::Ch4Set,
            5 => Register::Ch5Set,
            6 => Register::Ch6Set,
            7 => Register::Ch7Set,
            8 => Register::Ch8Set,
            _ => panic!("channel must be 1..=8"),
        };
        self.write_register(register, config.to_register()).await
    }

    /// Reads one frame in RDATAC mode. Call it after `/DRDY` goes low.
    pub async fn read_frame(&mut self) -> Result<Frame, Error<SPI::Error>> {
        // Transferring a zeroed buffer guarantees DIN stays low: any other
        // byte could be decoded as a command.
        let mut raw = [0; FRAME_LEN];
        self.spi
            .transaction(&mut [
                Operation::TransferInPlace(&mut raw),
                Operation::DelayNs(T_DECODE_NS),
            ])
            .await?;
        Frame::parse(&raw).ok_or(Error::InvalidFrame(raw[0]))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embassy_futures::block_on;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};
    use std::vec;

    #[test]
    fn decodes_known_ids() {
        assert_eq!(Model::from_id(0x90), Some(Model::Ads1294));
        assert_eq!(Model::from_id(0x92), Some(Model::Ads1298));
        assert_eq!(Model::from_id(0xD2), Some(Model::Ads1298R));
        assert_eq!(Model::from_id(0x92).map(Model::channels), Some(8));
    }

    #[test]
    fn rejects_unknown_ids() {
        for id in [0x00, 0xFF, 0x93, 0x82, 0xB2] {
            assert_eq!(Model::from_id(id), None, "id {id:#04x}");
        }
    }

    #[test]
    fn data_rate_config1() {
        assert_eq!(DataRate::Sps1k.config1(), 0x85);
        assert_eq!(DataRate::Sps1k.hz(), 1000);
        assert_eq!(DataRate::Sps32k.hz(), 32_000);
        assert_eq!(DataRate::Sps500.hz(), 500);
    }

    #[test]
    fn config3_bits() {
        let reference_only = Config3 {
            internal_reference: true,
            rld_buffer: false,
            rld_internal_reference: false,
        };
        assert_eq!(reference_only.to_register(), 0xC0);

        let rld_external_ref = Config3 {
            rld_buffer: true,
            ..reference_only
        };
        assert_eq!(rld_external_ref.to_register(), 0xC4);

        let rld_internal_ref = Config3 {
            rld_internal_reference: true,
            ..rld_external_ref
        };
        assert_eq!(rld_internal_ref.to_register(), 0xCC);
    }

    #[test]
    fn channel_config_bits() {
        let test_signal = ChannelConfig {
            power_down: false,
            gain: Gain::X1,
            input: ChannelInput::TestSignal,
        };
        assert_eq!(test_signal.to_register(), 0x15);

        let unused = ChannelConfig {
            power_down: true,
            gain: Gain::X6,
            input: ChannelInput::Shorted,
        };
        assert_eq!(unused.to_register(), 0x81);
    }

    fn raw_frame(status: [u8; 3], sample: [u8; 3]) -> [u8; FRAME_LEN] {
        let mut raw = [0; FRAME_LEN];
        raw[..3].copy_from_slice(&status);
        for bytes in raw[3..].as_chunks_mut::<3>().0 {
            *bytes = sample;
        }
        raw
    }

    #[test]
    fn parses_frame_with_sign_extension() {
        let cases = [
            ([0x00, 0x00, 0x01], 1),
            ([0x7F, 0xFF, 0xFF], 8_388_607),
            ([0xFF, 0xFF, 0xFF], -1),
            ([0x80, 0x00, 0x00], -8_388_608),
        ];
        for (bytes, expected) in cases {
            let frame = Frame::parse(&raw_frame([0xC0, 0x00, 0x00], bytes)).unwrap();
            assert_eq!(frame.samples, [expected; CHANNELS], "bytes {bytes:02x?}");
        }
    }

    #[test]
    fn parses_lead_off_status() {
        let frame = Frame::parse(&raw_frame([0xC1, 0x23, 0x45], [0; 3])).unwrap();
        assert_eq!(frame.lead_off_p(), 0x12);
        assert_eq!(frame.lead_off_n(), 0x34);
    }

    #[test]
    fn rejects_frame_without_header() {
        assert_eq!(Frame::parse(&raw_frame([0x00, 0x00, 0x00], [0; 3])), None);
        assert_eq!(Frame::parse(&raw_frame([0xFF, 0xFF, 0xFF], [0; 3])), None);
    }

    #[test]
    fn converts_to_microvolts() {
        let full_scale = Gain::X1.to_microvolts(8_388_607, VREF_VOLTS);
        assert!((full_scale - 2_400_000.0).abs() < 1.0);
        let one_mv_at_gain_6 = Gain::X6.to_microvolts(20_972, VREF_VOLTS);
        assert!((one_mv_at_gain_6 - 1000.0).abs() < 0.5);
    }

    #[test]
    fn reads_frame_clocking_zeros() {
        let raw = raw_frame([0xC0, 0x00, 0x00], [0xFF, 0xFF, 0xFE]);
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer_in_place(vec![0; FRAME_LEN], raw.to_vec()),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ];
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        let frame = block_on(ads.read_frame()).unwrap();
        assert_eq!(frame.samples, [-2; CHANNELS]);
        spi.done();
    }

    #[test]
    fn command_waits_for_decode() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x11]),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ];
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        block_on(ads.command(Command::Sdatac)).unwrap();
        spi.done();
    }

    #[test]
    fn reads_model_with_rreg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x20]),
            Transaction::delay(T_DECODE_NS),
            Transaction::write_vec(vec![0x00]),
            Transaction::delay(T_DECODE_NS),
            Transaction::read_vec(vec![0x92]),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ];
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        assert_eq!(block_on(ads.read_model()), Ok(Model::Ads1298));
        spi.done();
    }

    fn rreg_expectations(address: u8, value: u8) -> [Transaction<u8>; 8] {
        [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x20 | address]),
            Transaction::delay(T_DECODE_NS),
            Transaction::write_vec(vec![0x00]),
            Transaction::delay(T_DECODE_NS),
            Transaction::read_vec(vec![value]),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ]
    }

    #[test]
    fn reads_registers_one_by_one() {
        let expectations = [rreg_expectations(0x01, 0x85), rreg_expectations(0x02, 0x10)].concat();
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        let mut regs = [0; 2];
        block_on(ads.read_registers(Register::Config1, &mut regs)).unwrap();
        assert_eq!(regs, [0x85, 0x10]);
        spi.done();
    }

    #[test]
    fn reports_silent_device() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x20]),
            Transaction::delay(T_DECODE_NS),
            Transaction::write_vec(vec![0x00]),
            Transaction::delay(T_DECODE_NS),
            Transaction::read_vec(vec![0x00]),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ];
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        assert_eq!(block_on(ads.read_model()), Err(Error::UnknownId(0x00)));
        spi.done();
    }

    #[test]
    fn writes_register_with_wreg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x41]),
            Transaction::delay(T_DECODE_NS),
            Transaction::write_vec(vec![0x00]),
            Transaction::delay(T_DECODE_NS),
            Transaction::write_vec(vec![0x86]),
            Transaction::delay(T_DECODE_NS),
            Transaction::transaction_end(),
        ];
        let mut spi = Mock::new(&expectations);
        let mut ads = Ads1298::new(spi.clone());

        block_on(ads.write_register(Register::Config1, 0x86)).unwrap();
        spi.done();
    }
}
