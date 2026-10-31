//! Recording of the enabled channels to CSV files.

use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use protocol::setup::DATA_RATE;

use crate::live::{Update, enabled_channels};

/// At most this much is lost if the receiver dies while recording.
const FLUSH_PERIOD: Duration = Duration::from_secs(1);

/// Longer labels are cut: a mark names a moment, it is not a note.
const MAX_LABEL_CHARS: usize = 100;

/// What the web UI asks the device thread to do.
#[derive(Clone, Debug)]
pub enum Command {
    Start,
    Stop,
    /// Marks the newest recorded frame; an empty label gets a number.
    Mark(String),
}

/// Writes frames as CSV rows: `index,time_s,CH3_uV,...`.
pub struct CsvWriter<W: Write> {
    out: W,
    first_index: Option<u32>,
    last_index: Option<u32>,
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
            last_index: None,
            frames: 0,
        })
    }

    /// Seconds from the first recorded frame, by the conversion count: the
    /// same time base for the data and the marks.
    fn time_s(&self, index: u32) -> f64 {
        let first = self.first_index.unwrap_or(index);
        f64::from(index.wrapping_sub(first)) / f64::from(DATA_RATE.hz())
    }

    /// `samples` are the enabled channels in microvolts. Three decimals keep
    /// the full resolution: one LSB is 0.024 uV at gain 12.
    pub fn write(&mut self, index: u32, samples: &[f32]) -> io::Result<()> {
        self.first_index.get_or_insert(index);
        // From the conversion count, not the clock: packets arrive in
        // bursts, and lost frames show up as a jump in both columns.
        let time_s = self.time_s(index);
        write!(self.out, "{index},{time_s:.4}")?;
        for sample in samples {
            write!(self.out, ",{sample:.3}")?;
        }
        writeln!(self.out)?;
        self.last_index = Some(index);
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

/// `ch34_261031_024622_marks.csv` next to `ch34_261031_024622.csv`.
fn marks_path(data: &Path) -> PathBuf {
    let stem = data.file_stem().unwrap_or_default().to_string_lossy();
    data.with_file_name(format!("{stem}_marks.csv"))
}

/// A label fit for one CSV field and one line; `None` if nothing is left.
fn clean_label(label: &str) -> Option<String> {
    let label: String = label
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .take(MAX_LABEL_CHARS)
        .collect();
    let label = label.trim();
    (!label.is_empty()).then(|| label.to_owned())
}

/// Quotes a field that would otherwise split the row (RFC 4180).
fn csv_field(text: &str) -> String {
    if text.contains([',', '"']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_owned()
    }
}

struct Active {
    writer: CsvWriter<BufWriter<File>>,
    path: PathBuf,
    last_flush: Instant,
    marks: u32,
    /// Opened with the first mark, so a recording without marks has no
    /// empty file next to it.
    marks_file: Option<File>,
}

impl Active {
    fn write_mark(&mut self, index: u32, label: &str) -> io::Result<()> {
        let file = match &mut self.marks_file {
            Some(file) => file,
            None => {
                let mut file = File::options()
                    .write(true)
                    .create_new(true)
                    .open(marks_path(&self.path))?;
                writeln!(file, "index,time_s,label")?;
                self.marks_file.insert(file)
            }
        };
        // Unbuffered: marks are rare, and each one is worth more than the
        // second of data a crash may cost.
        let time_s = self.writer.time_s(index);
        writeln!(file, "{index},{time_s:.4},{}", csv_field(label))
    }
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

    /// Returns what the web UI should learn besides the new status.
    pub fn handle(&mut self, command: Command) -> Option<Update> {
        match command {
            Command::Start => self.start(),
            Command::Stop => self.stop(),
            Command::Mark(label) => return self.mark(&label),
        }
        None
    }

    /// Marks the newest recorded frame: the moment of the click, late by the
    /// delivery delay (~50-150 ms; more while the device catches up on a
    /// backlog).
    fn mark(&mut self, label: &str) -> Option<Update> {
        let active = self.active.as_mut()?;
        let Some(index) = active.writer.last_index else {
            println!("mark ignored: nothing recorded yet");
            return None;
        };
        active.marks += 1;
        let label = clean_label(label).unwrap_or_else(|| format!("метка {}", active.marks));
        if let Err(e) = active.write_mark(index, &label) {
            let active = self.active.take().expect("checked above");
            self.fail(&active, &e);
            return None;
        }
        println!("mark {:?} at frame {index}", label);
        Some(Update::Mark { index, label })
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
            marks: 0,
            marks_file: None,
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
        let (file, frames, marks) = match &self.active {
            Some(active) => (
                Some(active.path.display().to_string()),
                active.writer.frames,
                active.marks,
            ),
            None => (None, 0, 0),
        };
        Update::Recording {
            file,
            seconds: frames as f64 / f64::from(DATA_RATE.hz()),
            marks,
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
    fn cleans_and_quotes_labels() {
        assert_eq!(clean_label("  сжал\nкулак "), Some("сжал кулак".into()));
        assert_eq!(clean_label(" \t "), None);
        assert_eq!(
            clean_label(&"x".repeat(500)).unwrap().chars().count(),
            MAX_LABEL_CHARS
        );
        assert_eq!(csv_field("plain"), "plain");
        assert_eq!(csv_field(r#"a, "b""#), r#""a, ""b""""#);
    }

    #[test]
    fn marks_go_to_a_file_next_to_the_data() {
        let dir = std::env::temp_dir().join(format!("myocell-marks-{}", std::process::id()));
        let mut recording = Recording::new(&dir);
        // Not recording: there is nowhere to put a mark.
        assert_eq!(recording.handle(Command::Mark("x".into())), None);
        recording.handle(Command::Start);
        // Recording, but no frame yet: nothing to attach it to.
        assert_eq!(recording.handle(Command::Mark("x".into())), None);

        recording.write(1000, &[0.0, 0.0]);
        recording.write(1001, &[0.0, 0.0]);
        recording.write(1002, &[0.0, 0.0]);
        let mark = recording.handle(Command::Mark(String::new()));
        assert_eq!(
            mark,
            Some(Update::Mark {
                index: 1002,
                label: "метка 1".into()
            })
        );
        recording.handle(Command::Mark("сжал, сильно".into()));
        let Update::Recording { file, marks, .. } = recording.status() else {
            panic!("not a recording status");
        };
        assert_eq!(marks, 2);
        recording.handle(Command::Stop);

        let marks_file = marks_path(Path::new(&file.unwrap()));
        assert!(marks_file.to_string_lossy().ends_with("_marks.csv"));
        let text = fs::read_to_string(&marks_file).unwrap();
        assert_eq!(
            text,
            "index,time_s,label\n1002,0.0010,метка 1\n1002,0.0010,\"сжал, сильно\"\n"
        );
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
