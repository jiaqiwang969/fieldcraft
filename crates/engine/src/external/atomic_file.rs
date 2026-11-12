//! Native streaming model saves. Old small-model saves retain their original IO path.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn save_model(
    path: &std::path::Path,
    write: impl FnOnce(std::fs::File) -> crate::Result<std::fs::File>,
) -> crate::Result<(u64, Option<std::path::PathBuf>)> {
    use std::{
        fs::{self, File, OpenOptions},
        io::{Read, Write},
        path::{Path, PathBuf},
        sync::atomic::{AtomicU64, Ordering},
    };
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let error = |e: std::io::Error| crate::EngineError::Params(format!("ExternalArchiveSave: {e}"));
    let parent = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    fs::create_dir_all(parent).map_err(error)?;
    fn copy(source: &Path, target: &Path) -> std::io::Result<()> {
        let mut src = File::open(source)?;
        let mut dst = OpenOptions::new().create_new(true).write(true).open(target)?;
        let mut buffer = [0u8; 65536];
        let result = (|| {
            loop {
                let n = src.read(&mut buffer)?;
                if n == 0 {
                    break;
                }
                dst.write_all(&buffer[..n])?;
            }
            dst.sync_all()
        })();
        if result.is_err() {
            let _ = fs::remove_file(target);
        }
        result
    }
    struct Temp(PathBuf);
    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.0);
        }
    }
    let mut temp = None;
    for _ in 0..100 {
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        let p = parent.join(format!(".model-stream-{}-{n}.tmp", std::process::id()));
        match OpenOptions::new().create_new(true).read(true).write(true).open(&p) {
            Ok(f) => {
                temp = Some((Temp(p), f));
                break;
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(error(e)),
        }
    }
    let (temp, file) = temp.ok_or_else(|| crate::EngineError::Params("ExternalArchiveSave: temporary names exhausted".into()))?;
    let file = write(file)?;
    let bytes = file.metadata().map_err(error)?.len();
    if bytes > fieldcraft_io::MAX_FILE_BYTES {
        return Err(crate::EngineError::Params("ExternalArchiveLimit: project file".into()));
    }
    file.sync_all().map_err(error)?;
    drop(file);
    let mut kept = None;
    if path.try_exists().map_err(error)? {
        let format = fieldcraft_io::fs::format_of_file(path);
        if format != Some(fieldcraft_io::FORMAT_VERSION) {
            for target in fieldcraft_io::fs::version_backup_paths(path, format) {
                match copy(path, &target) {
                    Ok(()) => {
                        kept = Some(target);
                        break;
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                    Err(e) => return Err(error(e)),
                }
            }
            if kept.is_none() {
                return Err(crate::EngineError::Params("ExternalArchiveSave: version backup names exhausted".into()));
            }
        }
        let backup_temp = temp.0.with_extension("backup.tmp");
        let backup_guard = Temp(backup_temp.clone());
        copy(path, &backup_temp).map_err(error)?;
        fs::rename(&backup_temp, fieldcraft_io::fs::backup_path(path)).map_err(error)?;
        drop(backup_guard);
    }
    fs::rename(&temp.0, path).map_err(error)?;
    File::open(parent).and_then(|f| f.sync_all()).map_err(error)?;
    Ok((bytes, kept))
}
