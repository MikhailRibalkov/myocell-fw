//! Statistics of one report period, shared by the console and the web UI.

use serde::Serialize;

use crate::spectrum;

/// Mains frequency; its harmonics are measured separately.
const MAINS_HZ: f64 = 50.0;
const MAINS_HARMONICS: usize = 5;
/// Peaks are searched up to the Nyquist frequency of 2 kSPS / 2.
const PEAK_SEARCH_MAX_HZ: u32 = 500;

/// The link to the device during one period.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct LinkStats {
    pub frames: u32,
    pub packets: u32,
    pub bad_frames: u32,
    pub lost_frames: u64,
    pub duplicate_frames: u64,
    pub restarts: u64,
    pub connected: bool,
    pub connections: u32,
    /// Frames were lost within this period, so its spectrum is less reliable.
    pub gaps: bool,
}

/// One channel during one period. Microvolts at the electrodes.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ChannelStats {
    pub name: String,
    /// Mean: mostly the input offset; removed from everything below.
    pub offset_uv: f64,
    /// Largest |sample|, offset included: what decides saturation.
    pub peak_uv: f64,
    /// The input range is +-`full_scale_uv` at the configured gain.
    pub full_scale_uv: f64,
    pub rms_uv: f64,
    /// RMS of 50 Hz and its harmonics, from the fundamental up.
    pub mains_uv: Vec<f64>,
    /// RMS of everything but the mains harmonics.
    pub rest_uv: f64,
    /// Strongest spectral lines as (Hz, RMS).
    pub peaks: Vec<(u32, f64)>,
}

impl ChannelStats {
    pub fn new(name: String, samples: &[f64], sample_rate: f64, full_scale_uv: f64) -> Self {
        let n = samples.len() as f64;
        let offset_uv = samples.iter().sum::<f64>() / n;
        let peak_uv = samples.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
        let ac: Vec<f64> = samples.iter().map(|x| x - offset_uv).collect();
        let variance = ac.iter().map(|x| x * x).sum::<f64>() / n;

        let mains_uv: Vec<f64> = (1..=MAINS_HARMONICS)
            .map(|k| spectrum::tone_rms(&ac, sample_rate, MAINS_HZ * k as f64))
            .collect();
        // Powers of orthogonal components add up, so what is left without
        // the mains harmonics is the rest of the variance.
        let mains_power: f64 = mains_uv.iter().map(|rms| rms * rms).sum();
        let rest_uv = (variance - mains_power).max(0.0).sqrt();

        Self {
            name,
            offset_uv,
            peak_uv,
            full_scale_uv,
            rms_uv: variance.sqrt(),
            mains_uv,
            rest_uv,
            peaks: spectrum::peaks(&ac, sample_rate, PEAK_SEARCH_MAX_HZ, 3),
        }
    }

    /// One console line, as the receiver has always printed it.
    pub fn console_line(&self) -> String {
        let mains: Vec<String> = self
            .mains_uv
            .iter()
            .enumerate()
            .map(|(k, rms)| format!("{}:{rms:.1}", (k + 1) * MAINS_HZ as usize))
            .collect();
        let peaks: Vec<String> = self
            .peaks
            .iter()
            .map(|(f, rms)| format!("{f} Hz {rms:.1}"))
            .collect();
        format!(
            "  {}: rms {:7.1} | rest {:7.1} | mean {:+8.1} | mains [{}] | peaks [{}]  (uV)",
            self.name,
            self.rms_uv,
            self.rest_uv,
            self.offset_uv,
            mains.join(" "),
            peaks.join(", "),
        )
    }
}

impl LinkStats {
    pub fn console_line(&self) -> String {
        let gaps = if self.gaps { " | GAPS IN WINDOW" } else { "" };
        format!(
            "{} frames/s in {} packets | lost {} | duplicates {} | bad frames {} | tcp {}, connections {} | restarts {}{gaps}",
            self.frames,
            self.packets,
            self.lost_frames,
            self.duplicate_frames,
            self.bad_frames,
            if self.connected { "up" } else { "down" },
            self.connections,
            self.restarts,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 2000.0;

    fn tone(freq: f64, amplitude: f64) -> impl Fn(usize) -> f64 {
        move |i| amplitude * (2.0 * std::f64::consts::PI * freq * i as f64 / FS).sin()
    }

    fn stats(sample: impl Fn(usize) -> f64) -> ChannelStats {
        let samples: Vec<f64> = (0..2000).map(sample).collect();
        ChannelStats::new("CH3".into(), &samples, FS, 200_000.0)
    }

    #[test]
    fn separates_offset_and_mains() {
        let mains = tone(50.0, 100.0);
        let s = stats(|i| -10_000.0 + mains(i));
        assert!((s.offset_uv + 10_000.0).abs() < 1e-6, "{s:?}");
        // RMS of a sine is its amplitude / sqrt(2).
        assert!((s.mains_uv[0] - 100.0 / 2f64.sqrt()).abs() < 0.01, "{s:?}");
        assert!(s.rest_uv < 0.01, "{s:?}");
        // The peak includes the offset: that is what saturates the input.
        assert!((s.peak_uv - 10_100.0).abs() < 1.0, "{s:?}");
    }

    #[test]
    fn counts_other_tones_as_rest() {
        let s = stats(tone(120.0, 10.0));
        assert!((s.rest_uv - 10.0 / 2f64.sqrt()).abs() < 0.01, "{s:?}");
        assert!(s.mains_uv.iter().all(|&m| m < 0.01), "{s:?}");
        assert_eq!(s.peaks[0].0, 120);
    }
}
