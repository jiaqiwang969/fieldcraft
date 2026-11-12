//! Private bounded native storage for external chunks. One spool handle, no array cache.
use super::{ContractError, chunked::ChunkFile, package::FileSpec};
use fieldcraft_io::external_results::{ArchivedChunkedTrajectory, BlobDescriptor, BlobSource};
use sha2::{Digest, Sha256};
use std::io::{Read, Write};
fn err(e: impl std::fmt::Display) -> ContractError {
    ContractError::Provider(e.to_string())
}
pub(crate) fn copy_checked(reader: &mut dyn Read, out: &mut dyn Write, spec: &ChunkFile) -> Result<(), ContractError> {
    let mut buffer = [0u8; 65536];
    let mut left = spec.bytes;
    let mut hash = Sha256::new();
    while left > 0 {
        let n = left.min(buffer.len() as u64) as usize;
        let n = match reader.read(&mut buffer[..n]) {
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            r => r.map_err(err)?,
        };
        if n == 0 {
            return Err(ContractError::Invalid("ExternalSourceChanged: truncated"));
        }
        hash.update(&buffer[..n]);
        out.write_all(&buffer[..n]).map_err(err)?;
        left -= n as u64;
    }
    loop {
        match reader.read(&mut buffer[..1]) {
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Ok(0) => break,
            Ok(_) => return Err(ContractError::Invalid("ExternalSourceChanged: grew")),
            Err(e) => return Err(err(e)),
        }
    }
    let actual: String = hash.finalize().iter().map(|b| format!("{b:02x}")).collect();
    if actual != spec.sha256 {
        return Err(ContractError::ChecksumMismatch);
    }
    Ok(())
}
#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::*;
    use std::{
        fs::{self, File, OpenOptions},
        io::{Seek, SeekFrom},
        path::{Path, PathBuf},
        sync::{
            Arc, Mutex,
            atomic::{AtomicU64, Ordering},
        },
    };
    static NEXT: AtomicU64 = AtomicU64::new(0);
    fn unique_dir(parent: &Path) -> Result<PathBuf, ContractError> {
        for _ in 0..100 {
            let n = NEXT.fetch_add(1, Ordering::Relaxed);
            let p = parent.join(format!(".external-{}-{n}.tmp", std::process::id()));
            match fs::create_dir(&p) {
                Ok(()) => return Ok(p),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(err(e)),
            }
        }
        Err(ContractError::Invalid("ExternalStore: temporary names exhausted"))
    }
    fn sync_dir(path: &Path) -> Result<(), ContractError> {
        File::open(path).and_then(|f| f.sync_all()).map_err(err)
    }
    pub(crate) struct Stage {
        root: PathBuf,
        tmp: PathBuf,
        file: Arc<Mutex<File>>,
        descriptors: Vec<BlobDescriptor>,
        ranges: Vec<(u64, u64)>,
        total: u64,
        published: bool,
    }
    impl Stage {
        pub(crate) fn new(root: &Path) -> Result<Self, ContractError> {
            fs::create_dir_all(root).map_err(err)?;
            let tmp = unique_dir(root)?;
            let file = OpenOptions::new().read(true).write(true).create_new(true).open(tmp.join("blobs.data")).map_err(err)?;
            Ok(Self {
                root: root.to_path_buf(),
                tmp,
                file: Arc::new(Mutex::new(file)),
                descriptors: vec![],
                ranges: vec![],
                total: 0,
                published: false,
            })
        }
        pub(crate) fn append(&mut self, spec: &ChunkFile, reader: &mut dyn Read) -> Result<(), ContractError> {
            spec.validate(256 * 1024 * 1024)?;
            let total = self
                .total
                .checked_add(spec.bytes)
                .filter(|n| *n <= fieldcraft_io::MAX_FILE_BYTES)
                .ok_or(ContractError::Invalid("ExternalPackageLimit: spool"))?;
            let mut f = self.file.lock().map_err(|_| ContractError::Invalid("ExternalStore: file lock"))?;
            f.seek(SeekFrom::Start(self.total)).map_err(err)?;
            copy_checked(reader, &mut *f, spec)?;
            self.ranges.push((self.total, spec.bytes));
            self.descriptors.push(BlobDescriptor {
                index: self.descriptors.len() as u32,
                role: spec.role.clone(),
                bytes: spec.bytes,
                sha256: spec.sha256.clone(),
            });
            self.total = total;
            Ok(())
        }
        pub(crate) fn source(&self) -> Result<(Arc<dyn BlobSource>, Vec<BlobDescriptor>), ContractError> {
            self.file.lock().map_err(|_| ContractError::Invalid("ExternalStore: file lock"))?.sync_all().map_err(err)?;
            Ok((Arc::new(Spool { file: self.file.clone(), ranges: self.ranges.clone() }), self.descriptors.clone()))
        }
        pub(crate) fn publish(&mut self, digest: &str, record: &ArchivedChunkedTrajectory) -> Result<(), ContractError> {
            if !fieldcraft_io::identity::digest_valid(digest) {
                return Err(ContractError::Invalid("result identifier digest"));
            }
            // A cross-process reservation serializes cooperative writers for this ID.
            let lock = self.root.join(format!(".{digest}.lock"));
            let held = OpenOptions::new().create_new(true).write(true).open(&lock).map_err(err)?;
            struct Lock(std::fs::File, PathBuf);
            impl Drop for Lock {
                fn drop(&mut self) {
                    let _ = self.0.sync_all();
                    let _ = fs::remove_file(&self.1);
                }
            }
            let _lock = Lock(held, lock);
            let target = self.root.join(digest);
            if target.try_exists().map_err(err)? {
                return Err(ContractError::Invalid("ExternalStore: result already exists"));
            }
            for (name, bytes) in
                [("manifest.json", record.manifest_json()), ("admission.json", record.admission_json()), ("context.json", record.context_json())]
            {
                let mut f = OpenOptions::new().create_new(true).write(true).open(self.tmp.join(name)).map_err(err)?;
                f.write_all(bytes).map_err(err)?;
                f.sync_all().map_err(err)?;
            }
            let mut f = OpenOptions::new().create_new(true).write(true).open(self.tmp.join("blobs.json")).map_err(err)?;
            serde_json::to_writer(&mut f, record.blobs()).map_err(err)?;
            f.sync_all().map_err(err)?;
            sync_dir(&self.tmp)?;
            fs::rename(&self.tmp, &target).map_err(err)?;
            self.published = true;
            sync_dir(&self.root)
        }
    }
    impl Drop for Stage {
        fn drop(&mut self) {
            if !self.published {
                let _ = fs::remove_dir_all(&self.tmp);
            }
        }
    }
    struct Spool {
        file: Arc<Mutex<File>>,
        ranges: Vec<(u64, u64)>,
    }
    struct Range {
        file: Arc<Mutex<File>>,
        pos: u64,
        left: u64,
    }
    impl Read for Range {
        fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
            let n = b.len().min(self.left.min(usize::MAX as u64) as usize);
            if n == 0 {
                return Ok(0);
            }
            let mut f = self.file.lock().map_err(|_| std::io::Error::other("external spool lock"))?;
            f.seek(SeekFrom::Start(self.pos))?;
            let n = f.read(&mut b[..n])?;
            self.pos += n as u64;
            self.left -= n as u64;
            Ok(n)
        }
    }
    impl BlobSource for Spool {
        fn open(&self, i: u32) -> fieldcraft_io::Result<Box<dyn Read + '_>> {
            let &(pos, left) = self.ranges.get(i as usize).ok_or_else(|| fieldcraft_io::Error::Invalid("ExternalArchiveMissingBlob".into()))?;
            Ok(Box::new(Range { file: self.file.clone(), pos, left }))
        }
    }
    pub(crate) fn read_original_model(reader: &mut dyn Read, spec: &FileSpec) -> Result<fieldcraft_io::ModelFile, ContractError> {
        let root = unique_dir(&std::env::temp_dir())?;
        struct Temp(PathBuf);
        impl Drop for Temp {
            fn drop(&mut self) {
                let _ = fs::remove_dir_all(&self.0);
            }
        }
        let _guard = Temp(root.clone());
        let path = root.join("input.fieldcraft");
        let mut file = OpenOptions::new().create_new(true).write(true).open(&path).map_err(err)?;
        copy_checked(
            reader,
            &mut file,
            &ChunkFile { locator: spec.locator.clone(), role: spec.role.clone(), sha256: spec.sha256.clone(), bytes: spec.bytes },
        )?;
        drop(file);
        fieldcraft_io::read_path_streamed(&path).map_err(err)
    }
}
#[cfg(not(target_arch = "wasm32"))]
pub(crate) use native::{Stage, read_original_model};
#[cfg(target_arch = "wasm32")]
pub(crate) fn read_original_model(_: &mut dyn Read, _: &FileSpec) -> Result<fieldcraft_io::ModelFile, ContractError> {
    Err(ContractError::Unsupported("chunked archives require a native streaming filesystem"))
}
