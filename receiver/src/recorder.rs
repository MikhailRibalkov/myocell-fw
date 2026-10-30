//! Recording of the enabled channels to CSV files.

use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use protocol::setup::DATA_RATE;

use crate::live::{Update, enabled_channels};

/// At most this much is lost if the receiver dies while recording.
const FLUSH_PERIOD: Duration = Duration::from_secs(1);

/// What the web UI asks the device thread to do.
#[derive(Clone, Copy, Debug)]
pub enum Command {
    Start,
    Stop,
}

/// Writes frames as CSV rows: `index,time_s,CH3_uV,...`.
pub struct CsvWriter<W: Write> {
    out: W,
    first_index: Option<u32>,
    frames: u64,
}

impl<W: Write> CsvWriter<W> {
    pub fn new(mut out: W) -> io::Result<Self> {
        write!(out, "index,time_s")?;
        for i in enabled_channels() {
            write!(out, ",CH{}_uV", i + 1)?;
        }
        writeln!(out)?;
        Ok(Self {
            out,
            first_index: None,
            frames: 0,
        })
    }

    /// `samples` are the enabled channels in microvolts. Three decimals keep
    /// the full resolution: one LSB is 0.024 uV at gain 12.
    pub fn write(&mut self, index: u32, samples: &[f32]) -> io::Result<()> {
        let first = *self.first_index.get_or_insert(index);
        // From the conversion count, not the clock: packets arrive in
        // bursts, and lost frames show up as a jump in both columns.
        let time_s = f64::from(index.wrapping_sub(first)) / f64::from(DATA_RATE.hz());
        write!(self.out, "{index},{time_s:.4}")?;
        for sample in samples {
            write!(self.out, ",{sample:.3}")?;
        }
        writeln!(self.out)?;
        self.frames += 1;
        Ok(())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }
}

/// `ch34_261031_024622.csv`: the enabled channels, then the local start time
/// to the second. Channel numbers are single digits (1..=8), so they need no
/// separator.
fn file_name(start: chrono::NaiveDateTime) -> String {
    let channels: String = enabled_channels().map(|i| (i + 1).to_string()).collect();
    format!("ch{channels}_{}.csv", start.format("%y%m%d_%H%M%S"))
}

struct Active {
    writer: CsvWriter<BufWriter<File>>,
    path: PathBuf,
    last_flush: Instant,
}

/// The recording state, owned by the device thread: it sees every frame in
/// order, so nothing between the device and the file can drop data.
pub struct Recording {
    dir: PathBuf,
    active: Option<Active>,
    /// Why the last recording ended on its own; shown until the next start.
    error: Option<String>,
}

impl Recording {
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self {
            dir: dir.into(),
            active: None,
            error: None,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }

    pub fn handle(&mut self, command: Command) {
        match command {
            Command::Start => self.start(),
            Command::Stop => self.stop(),
        }
    }

    fn start(&mut self) {
        if self.active.is_some() {
            return;
        }
        self.error = None;
        match self.open() {
            Ok(active) => {
                println!("recording to {}", active.path.display());
                self.active = Some(active);
            }
            Err(e) => self.error = Some(format!("cannot start: {e}")),
        }
    }

    fn open(&self) -> io::Result<Active> {
        fs::create_dir_all(&self.dir)?;
        let path = self.dir.join(file_name(chrono::Local::now().naive_local()));
        // Never overwrite: two starts within one second fail instead.
        let file = File::options().write(true).create_new(true).open(&path)?;
        Ok(Active {
            writer: CsvWriter::new(BufWriter::new(file))?,
            path,
            last_flush: Instant::now(),
        })
    }

    fn stop(&mut self) {
        if let Some(mut active) = self.active.take() {
            match active.writer.flush() {
                Ok(()) => println!(
                    "recorded {} frames to {}",
                    active.writer.frames,
                    active.path.display()
                ),
                Err(e) => self.fail(&active, &e),
            }
        }
    }

    /// The device restarted and counts from zero again: the time column
    /// would break, so the recording continues in a new file.
    pub fn split(&mut self) {
        if self.is_active() {
            self.stop();
            self.start();
        }
    }

    /// `samples` are the enabled channels in microvolts.
    pub fn write(&mut self, index: u32, samples: &[f32]) {
        let Some(active) = &mut self.active else {
            return;
        };
        if let Err(e) = active.writer.write(index, samples) {
            let active = self.active.take().expect("checked above");
            self.fail(&active, &e);
        }
    }

    /// Flushes now and then, so a crash loses at most [`FLUSH_PERIOD`].
    pub fn flush_due(&mut self) {
        let Some(active) = &mut self.active else {
            return;
        };
        if active.last_flush.elapsed() < FLUSH_PERIOD {
            return;
        }
        active.last_flush = Instant::now();
        if let Err(e) = active.writer.flush() {
            let active = self.active.take().expect("checked above");
            self.fail(&active, &e);
        }
    }

    /// Stops recording, not receiving: the device still gets its acks and
    /// the plots keep running.
    fn fail(&mut self, active: &Active, error: &io::Error) {
        let message = format!("{}: {error}", active.path.display());
        println!("recording stopped: {message}");
        self.error = Some(message);
    }

    pub fn status(&self) -> Update {
        let (file, frames) = match &self.active {
            Some(active) => (
                Some(active.path.display().to_string()),
                active.writer.frames,
            ),
            None => (None, 0),
        };
        Update::Recording {
            file,
            seconds: frames as f64 / f64::from(DATA_RATE.hz()),
            error: self.error.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_files_by_channels_and_time() {
        let start = chrono::NaiveDate::from_ymd_opt(2026, 10, 31)
            .unwrap()
            .and_hms_opt(2, 46, 22)
            .unwrap();
        assert_eq!(file_name(start), "ch34_261031_024622.csv");
    }

    #[test]
    fn writes_header_and_rows() {
        let mut writer = CsvWriter::new(Vec::new()).unwrap();
        writer.write(100, &[1.0, -2.5]).unwrap();
        writer.write(101, &[0.0125, 3.0]).unwrap();
        let text = String::from_utf8(writer.out).unwrap();
        assert_eq!(
            text,
            "index,time_s,CH3_uV,CH4_uV\n\
             100,0.0000,1.000,-2.500\n\
             101,0.0005,0.013,3.000\n"
        );
    }

    #[test]
    fn time_follows_the_index_across_gaps() {
        let mut writer = CsvWriter::new(Vec::new()).unwrap();
        writer.write(10, &[0.0, 0.0]).unwrap();
        // 1990 conversions were lost: 1 s later by the device clock.
        writer.write(2010, &[0.0, 0.0]).unwrap();
        let text = String::from_utf8(writer.out).unwrap();
        assert!(text.ends_with("2010,1.0000,0.000,0.000\n"), "{text}");
        assert_eq!(writer.frames, 2);
    }

    #[test]
    fn time_survives_the_index_wrap() {
        let mut writer = CsvWriter::new(Vec::new()).unwrap();
        writer.write(u32::MAX, &[0.0, 0.0]).unwrap();
        writer.write(1, &[0.0, 0.0]).unwrap();
        let text = String::from_utf8(writer.out).unwrap();
        assert!(text.ends_with("1,0.0010,0.000,0.000\n"), "{text}");
    }

    #[test]
    fn records_to_a_new_file_and_stops() {
        let dir = std::env::temp_dir().join(format!("myocell-test-{}", std::process::id()));
        let mut recording = Recording::new(&dir);
        recording.handle(Command::Start);
        assert!(recording.is_active());
        recording.write(5, &[1.0, 2.0]);
        let Update::Recording { file, .. } = recording.status() else {
            panic!("not a recording status");
        };
        let path = PathBuf::from(file.unwrap());
        recording.handle(Command::Stop);
        assert!(!recording.is_active());

        let text = fs::read_to_string(&path).unwrap();
        assert_eq!(text, "index,time_s,CH3_uV,CH4_uV\n5,0.0000,1.000,2.000\n");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_failed_start_is_reported() {
        // A file where the directory should be: create_dir_all fails.
        let blocker = std::env::temp_dir().join(format!("myocell-blocker-{}", std::process::id()));
        fs::write(&blocker, b"").unwrap();
        let mut recording = Recording::new(&blocker);
        recording.handle(Command::Start);
        assert!(!recording.is_active());
        let Update::Recording { error, .. } = recording.status() else {
            panic!("not a recording status");
        };
        assert!(error.unwrap().starts_with("cannot start"));
        fs::remove_file(&blocker).unwrap();
    }
}
