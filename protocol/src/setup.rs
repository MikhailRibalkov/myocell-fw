//! Analog front-end setup, shared so that the receiver converts samples
//! with the same gains the firmware configured.

use ads1298::{CHANNELS, ChannelConfig, ChannelInput, Config3, DataRate, Gain};

/// The sinc filter of the ADS has its -3 dB point at 0.262 * f_DR: 524 Hz
/// here, above the ~350 Hz analog low-pass of the board, so the whole
/// surface EMG band (20..450 Hz) passes.
pub const DATA_RATE: DataRate = DataRate::Sps2k;

/// CH3 and CH4 are AC-coupled (2.2 uF) to the on-board electrode pads, so
/// electrode DC offsets never reach the ADC and the +-200 mV range of
/// gain 12 is plenty for EMG while keeping input-referred noise low.
const EMG: ChannelConfig = ChannelConfig {
    power_down: false,
    gain: Gain::X12,
    input: ChannelInput::Normal,
};

/// CH1/CH2 go to the external connector J1, CH5..CH8 are tied to AVDD.
const UNUSED: ChannelConfig = ChannelConfig {
    power_down: true,
    gain: Gain::X1,
    input: ChannelInput::Shorted,
};

/// `CHnSET` for channels 1..=8 (index = channel - 1).
pub const CHANNEL_CONFIG: [ChannelConfig; CHANNELS] =
    [UNUSED, UNUSED, EMG, EMG, UNUSED, UNUSED, UNUSED, UNUSED];

/// Channels whose common mode drives the right-leg electrode (RLD pad J8),
/// bit n = channel n + 1: CH3 and CH4 on both inputs.
pub const RLD_SENSE: u8 = 1 << 2 | 1 << 3;

/// `RLDREF` is tied to GNDA on this board revision (R12 = 0 ohm), so the
/// internal mid-supply RLD reference stays off.
pub const CONFIG3: Config3 = Config3 {
    internal_reference: true,
    rld_buffer: true,
    rld_internal_reference: false,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rld_senses_only_enabled_channels() {
        for (i, config) in CHANNEL_CONFIG.iter().enumerate() {
            let sensed = RLD_SENSE & (1 << i) != 0;
            assert_eq!(sensed, !config.power_down, "CH{}", i + 1);
        }
    }
}
