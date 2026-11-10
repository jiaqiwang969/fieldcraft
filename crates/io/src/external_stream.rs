//! File-backed stored ZIP blobs. At most one 64 KiB copy buffer is live per traversal.
use crate::{Error, Result, external_results::*, results, *};
use std::{
    fs::File,
    io::{Read, Seek, SeekFrom},
    sync::{Arc, Mutex},
};
pub(crate) fn blob_name_valid(n: &str) -> bool {
    n.strip_prefix("external-").and_then(|s| s.strip_suffix(".bin")).is_some_and(identity::digest_valid)
}
pub(crate) fn collect<'a>(extras: &'a Extras<'a>, model: &Model, g: &Geometry) -> Result<BTreeMap<String, (&'a ArchivedChunkedTrajectory, u32)>> {
    let mut blobs: BTreeMap<String, (&ArchivedChunkedTrajectory, u32)> = BTreeMap::new();
    let mut metadata = 0u64;
    for (_, stored) in &extras.results {
        if let results::SolvedRef::ExternalChunked(a) = stored.result {
            a.validate_binding(model, g, &extras.run)?;
            let mut size = DigestWriter::default();
            crate::codec::encode_to(a.record.as_ref(), &mut size).map_err(|e| Error::Invalid(e.to_string()))?;
            metadata = metadata.checked_add(size.part().bytes).ok_or_else(|| Error::Invalid("ExternalArchiveLimit: metadata overflow".into()))?;
            if metadata > crate::external_chunked::MAX_RECORD_BYTES {
                return invalid("ExternalArchiveLimit: metadata working set");
            }

            if model.step(a.archive_key()).is_some() {
                return invalid("ExternalArchiveDuplicateKey: native step");
            }
            for d in a.blobs() {
                let name = format!("external-{}.bin", d.sha256);
                if let Some((prior, i)) = blobs.get(&name) {
                    let prior: &ArchivedChunkedTrajectory = prior;
                    if prior.blobs()[*i as usize].bytes != d.bytes {
                        return invalid("ExternalArchiveDigestMismatch: duplicate size");
                    }
                    // Deduplication must not hide a changed earlier source with the same claim.
                    crate::external_chunked::copy_blob(prior, *i, &mut std::io::sink())?;
                }
                blobs.insert(name, (a, d.index));
            }
        }
    }
    Ok(blobs)
}
pub fn read_path_streamed(path: &std::path::Path) -> Result<ModelFile> {
    let f = File::open(path).map_err(|e| Error::Io(e.to_string()))?;
    let len = f.metadata().map_err(|e| Error::Io(e.to_string()))?.len();
    let source = Arc::new(Mutex::new(f.try_clone().map_err(|e| Error::Io(e.to_string()))?));
    crate::read_archive(f, usize::try_from(len).map_err(|_| Error::Invalid("ExternalArchiveLimit: file".into()))?, Some(source)).map(|(m, _)| m)
}
struct FileBlobs {
    file: Arc<Mutex<File>>,
    ranges: Vec<(u64, u64)>,
}
struct RangeReader {
    file: Arc<Mutex<File>>,
    pos: u64,
    left: u64,
}
impl Read for RangeReader {
    fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
        let n = b.len().min(self.left.min(usize::MAX as u64) as usize);
        if n == 0 {
            return Ok(0);
        }
        let mut f = self.file.lock().map_err(|_| std::io::Error::other("external archive file lock"))?;
        f.seek(SeekFrom::Start(self.pos))?;
        let n = f.read(&mut b[..n])?;
        self.pos += n as u64;
        self.left -= n as u64;
        Ok(n)
    }
}
impl BlobSource for FileBlobs {
    fn open(&self, index: u32) -> Result<Box<dyn Read + '_>> {
        let &(pos, left) = self.ranges.get(index as usize).ok_or_else(|| Error::Invalid("ExternalArchiveMissingBlob".into()))?;
        Ok(Box::new(RangeReader { file: self.file.clone(), pos, left }))
    }
}
pub(crate) fn attach<R: Read + Seek>(
    results: &mut [results::Stored],
    zip: &mut zip::ZipArchive<R>,
    listed: &BTreeMap<String, Part>,
    source: Option<Arc<Mutex<File>>>,
    model: &Model,
    g: &Geometry,
    run: &Run,
) -> Result<()> {
    let mut used = BTreeSet::new();
    for s in results {
        if let results::Solved::ExternalChunked(a) = &mut s.result {
            a.validate_binding(model, g, run)?;
            if model.step(a.archive_key()).is_some() {
                return invalid("ExternalArchiveDuplicateKey: native step");
            }
            let file = source.clone().ok_or_else(|| Error::Invalid("ExternalArchiveMissingBlob: use read_path_streamed".into()))?;
            let mut ranges = Vec::new();
            for d in a.blobs() {
                let name = format!("external-{}.bin", d.sha256);
                let part = listed.get(&name).ok_or_else(|| Error::Invalid("ExternalArchiveMissingBlob".into()))?;
                if part.bytes != d.bytes || part.sha256 != d.sha256 {
                    return invalid("ExternalArchiveDigestMismatch: descriptor");
                }
                let mut f = zip.by_name(&name).map_err(|_| Error::Invalid("ExternalArchiveMissingBlob".into()))?;
                if f.compression() != zip::CompressionMethod::Stored || f.size() != d.bytes {
                    return invalid("ExternalArchiveLimit: compressed or wrong size");
                }
                let pos = f.data_start().ok_or_else(|| Error::Invalid("ExternalArchiveMissingBlob: file offset".into()))?;
                let mut hash = Sha256::new();
                let mut buf = [0u8; COPY_BUFFER_BYTES];
                let mut count = 0u64;
                loop {
                    let n = f.read(&mut buf).map_err(|e| Error::Invalid(e.to_string()))?;
                    if n == 0 {
                        break;
                    }
                    count += n as u64;
                    if count > d.bytes {
                        return invalid("ExternalArchiveLimit: blob");
                    };
                    hash.update(&buf[..n]);
                }
                if count != d.bytes || crate::external_chunked::hex(&hash.finalize()) != d.sha256 {
                    return invalid("ExternalArchiveDigestMismatch: blob");
                }
                ranges.push((pos, d.bytes));
                used.insert(name);
            }
            a.source = Arc::new(FileBlobs { file, ranges });
        }
    }
    if listed.keys().any(|n| blob_name_valid(n) && !used.contains(n)) {
        return invalid("ExternalArchiveMissingBlob: unreferenced member");
    }
    Ok(())
}

#[derive(Default)]
pub(crate) struct DigestWriter {
    hash: Sha256,
    bytes: u64,
}
impl std::io::Write for DigestWriter {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.bytes = self.bytes.checked_add(b.len() as u64).ok_or_else(|| std::io::Error::other("size overflow"))?;
        self.hash.update(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl DigestWriter {
    pub(crate) fn part(self) -> Part {
        Part { bytes: self.bytes, sha256: crate::external_chunked::hex(&self.hash.finalize()) }
    }
}

pub(crate) fn read_results<R: Read + Seek>(zip: &mut zip::ZipArchive<R>, listed: &BTreeMap<String, Part>) -> Result<Option<results::Decoded>> {
    let Some(d) = listed.get(results::PART) else { return Ok(None) };
    if d.bytes > crate::MAX_FILE_BYTES {
        return invalid("results limit");
    }
    {
        let mut f = zip.by_name(results::PART).map_err(|e| Error::Invalid(e.to_string()))?;
        let mut digest = DigestWriter::default();
        let mut buf = [0; COPY_BUFFER_BYTES];
        loop {
            let n = f.read(&mut buf).map_err(|e| Error::Invalid(e.to_string()))?;
            if n == 0 {
                break;
            }
            std::io::Write::write_all(&mut digest, &buf[..n]).map_err(|e| Error::Io(e.to_string()))?;
            if digest.bytes > d.bytes {
                return invalid("results length");
            }
        }
        let p = digest.part();
        if p.bytes != d.bytes || p.sha256 != d.sha256 {
            return invalid("ExternalArchiveDigestMismatch: results part");
        }
    }
    let mut f = zip.by_name(results::PART).map_err(|e| Error::Invalid(e.to_string()))?;
    let mut r = std::io::BufReader::with_capacity(COPY_BUFFER_BYTES, &mut f);
    Ok(Some(results::decode_stream(&mut r, d.bytes)?))
}
