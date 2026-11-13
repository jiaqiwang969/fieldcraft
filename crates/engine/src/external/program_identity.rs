//! Identity of the receiving executable, not an untrusted result-file reader.
//! The program is streamed through one fixed buffer, independently of JSON/field caps.
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::{self, Read};
use std::sync::OnceLock;

const BUFFER_BYTES: usize = 64 * 1024;

static PROGRAM_SHA256: OnceLock<Result<String, String>> = OnceLock::new();

fn hash_exact(reader: &mut impl Read, expected_bytes: u64) -> io::Result<String> {
    let mut buffer = [0_u8; BUFFER_BYTES];
    let mut digest = Sha256::new();
    let mut remaining = expected_bytes;
    while remaining != 0 {
        let take = remaining.min(BUFFER_BYTES as u64) as usize;
        let slice = buffer.get_mut(..take).ok_or_else(|| io::Error::other("program hash buffer"))?;
        let n = match reader.read(slice) {
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            result => result?,
        };
        if n == 0 {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "receiving executable changed size"));
        }
        digest.update(buffer.get(..n).ok_or_else(|| io::Error::other("program hash read size"))?);
        remaining -= n as u64;
    }
    // A growing file cannot turn this into an unbounded read loop.
    loop {
        match reader.read(&mut buffer[..1]) {
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Ok(0) => break,
            Ok(_) => return Err(io::Error::other("receiving executable changed size")),
            Err(e) => return Err(e),
        }
    }
    Ok(digest.finalize().iter().map(|b| format!("{b:02x}")).collect())
}

fn hash_file(mut file: File) -> io::Result<String> {
    let before = file.metadata()?;
    if !before.is_file() {
        return Err(io::Error::other("receiving executable is not a regular file"));
    }
    let digest = hash_exact(&mut file, before.len())?;
    let after = file.metadata()?;
    if before.len() != after.len() || before.modified()? != after.modified()? {
        return Err(io::Error::other("receiving executable changed during identity measurement"));
    }
    Ok(digest)
}

fn measure_current_program_sha256() -> io::Result<String> {
    // Linux exposes the running image even after its pathname has been replaced.
    #[cfg(target_os = "linux")]
    let file = File::open("/proc/self/exe")?;
    #[cfg(not(target_os = "linux"))]
    let file = File::open(std::env::current_exe()?)?;
    hash_file(file)
}

fn cached_program_sha256(cache: &OnceLock<Result<String, String>>, measure: impl FnOnce() -> io::Result<String>) -> io::Result<String> {
    let cached = cache.get_or_init(|| measure().map_err(|cause| cause.to_string()));
    match cached {
        Ok(sha256) => Ok(sha256.clone()),
        Err(cause) => Err(io::Error::other(cause.clone())),
    }
}

pub(crate) fn current_program_sha256() -> io::Result<String> {
    cached_program_sha256(&PROGRAM_SHA256, measure_current_program_sha256)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Cursor, Write};

    #[test]
    fn program_digest_matches_public_digest_with_short_and_interrupted_reads() {
        struct Short<'a> {
            bytes: &'a [u8],
            interrupted: bool,
            largest_request: usize,
        }
        impl Read for Short<'_> {
            fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
                self.largest_request = self.largest_request.max(out.len());
                if !self.interrupted {
                    self.interrupted = true;
                    return Err(io::ErrorKind::Interrupted.into());
                }
                let n = out.len().min(7).min(self.bytes.len());
                out[..n].copy_from_slice(&self.bytes[..n]);
                self.bytes = &self.bytes[n..];
                Ok(n)
            }
        }
        let data = vec![123; 3 * BUFFER_BYTES + 9];
        let mut reader = Short { bytes: &data, interrupted: false, largest_request: 0 };
        assert_eq!(hash_exact(&mut reader, data.len() as u64).unwrap(), fieldcraft_io::sha256_hex(&data));
        assert!(reader.largest_request <= BUFFER_BYTES);
    }

    #[test]
    fn program_digest_refuses_truncation_growth_and_read_failure() {
        assert!(hash_exact(&mut Cursor::new(b"ab"), 3).is_err());
        assert!(hash_exact(&mut Cursor::new(b"abcd"), 3).is_err());
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
                Err(io::Error::other("source failure"))
            }
        }
        assert!(hash_exact(&mut Broken, 3).is_err());
    }

    #[test]
    fn program_file_larger_than_external_json_cap_is_streamed() {
        // Real file >256MiB; production external-file caps must still refuse it.
        let root = std::env::temp_dir().join(format!("fieldcraft-program-sha-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        let path = root.join("oversize-program");
        let length = super::super::context::MAX_BYTES + 17;
        let mut f = File::create(&path).unwrap();
        f.set_len(length).unwrap();
        f.write_all(b"identity-regression").unwrap();
        f.sync_all().unwrap();
        drop(f);
        let actual = hash_file(File::open(&path).unwrap()).unwrap();
        struct HashWriter(Sha256);
        impl Write for HashWriter {
            fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
                self.0.update(bytes);
                Ok(bytes.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let mut reference = HashWriter(Sha256::new());
        assert_eq!(std::io::copy(&mut File::open(&path).unwrap(), &mut reference).unwrap(), length);
        assert_eq!(actual, reference.0.finalize().iter().map(|b| format!("{b:02x}")).collect::<String>());
        assert!(fieldcraft_io::fs::read_capped(&path, super::super::context::MAX_BYTES).is_err());
        std::fs::remove_file(path).unwrap();
        std::fs::remove_dir(root).unwrap();
    }

    #[test]
    fn program_digest_is_measured_once_per_process() {
        // The production cache is process-wide.  Use the same cache helper with a local
        // counter so this test proves that the second lookup does not read the executable.
        let cache = OnceLock::new();
        let reads = std::cell::Cell::new(0);
        let measure = || {
            reads.set(reads.get() + 1);
            Ok("a".repeat(64))
        };
        assert_eq!(cached_program_sha256(&cache, measure).unwrap(), "a".repeat(64));
        assert_eq!(
            cached_program_sha256(&cache, || {
                reads.set(reads.get() + 1);
                Ok("b".repeat(64))
            })
            .unwrap(),
            "a".repeat(64)
        );
        assert_eq!(reads.get(), 1);
    }
}
