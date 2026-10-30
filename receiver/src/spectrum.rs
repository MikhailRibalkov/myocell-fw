//! Single-frequency analysis of a block of samples.

use std::f64::consts::{PI, SQRT_2};

/// RMS of the sinusoidal component of `samples` at `freq` Hz, in the units
/// of `samples`. The input should have its mean removed.
///
/// Goertzel algorithm: one DFT bin in O(N) with a single multiply per
/// sample, for any (also non-integer) frequency. A sine of amplitude A
/// gives |X| = A * N / 2, so its RMS is sqrt(2) * |X| / N.
pub fn tone_rms(samples: &[f64], sample_rate: f64, freq: f64) -> f64 {
    let omega = 2.0 * PI * freq / sample_rate;
    let coeff = 2.0 * omega.cos();
    let (mut s1, mut s2) = (0.0, 0.0);
    for &x in samples {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    // |X|^2 from the last two states of the recurrence.
    let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    SQRT_2 * power.max(0.0).sqrt() / samples.len() as f64
}

/// The `count` strongest local maxima of the spectrum between 1 Hz and
/// `max_freq`, on a 1 Hz grid, strongest first: `(frequency, rms)`.
pub fn peaks(samples: &[f64], sample_rate: f64, max_freq: u32, count: usize) -> Vec<(u32, f64)> {
    let spectrum: Vec<f64> = (0..=max_freq + 1)
        .map(|f| tone_rms(samples, sample_rate, f64::from(f)))
        .collect();
    let mut maxima: Vec<(u32, f64)> = (1..=max_freq)
        .map(|f| (f, spectrum[f as usize]))
        .filter(|&(f, rms)| rms > spectrum[f as usize - 1] && rms >= spectrum[f as usize + 1])
        .collect();
    maxima.sort_by(|a, b| b.1.total_cmp(&a.1));
    maxima.truncate(count);
    maxima
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 2000.0;

    fn sine(freq: f64, amplitude: f64) -> impl Fn(usize) -> f64 {
        move |i| amplitude * (2.0 * PI * freq * i as f64 / RATE).sin()
    }

    fn signal(n: usize, parts: &[(f64, f64)]) -> Vec<f64> {
        (0..n)
            .map(|i| parts.iter().map(|&(f, a)| sine(f, a)(i)).sum())
            .collect()
    }

    #[test]
    fn measures_tone_rms() {
        let x = signal(2000, &[(50.0, 100.0)]);
        let rms = tone_rms(&x, RATE, 50.0);
        assert!((rms - 100.0 / SQRT_2).abs() < 0.01, "{rms}");
    }

    #[test]
    fn separates_tones() {
        let x = signal(2000, &[(50.0, 100.0), (150.0, 10.0)]);
        assert!((tone_rms(&x, RATE, 150.0) - 10.0 / SQRT_2).abs() < 0.01);
        assert!(tone_rms(&x, RATE, 100.0) < 0.01);
    }

    #[test]
    fn tolerates_uneven_window() {
        // Windows are 1960..2040 samples: the bin is then off-center.
        let x = signal(1960, &[(50.0, 100.0)]);
        let rms = tone_rms(&x, RATE, 50.0);
        assert!((rms - 100.0 / SQRT_2).abs() < 1.0, "{rms}");
    }

    #[test]
    fn finds_strongest_peaks() {
        let x = signal(2000, &[(50.0, 100.0), (150.0, 30.0), (273.0, 10.0)]);
        let found: Vec<u32> = peaks(&x, RATE, 500, 3).iter().map(|p| p.0).collect();
        assert_eq!(found, [50, 150, 273]);
    }
}
