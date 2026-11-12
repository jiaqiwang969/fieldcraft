use fieldcraft_io::{external_results::*, identity::*, results::*, *};
use fieldcraft_model::Model;
use std::{
    io::{Cursor, Read},
    sync::Arc,
};
struct Bytes(Vec<u8>);
impl BlobSource for Bytes {
    fn open(&self, _: u32) -> fieldcraft_io::Result<Box<dyn Read + '_>> {
        Ok(Box::new(Cursor::new(&self.0)))
    }
}
fn sample(bytes: Vec<u8>) -> (Model, ArchivedChunkedTrajectoryParts) {
    let m = Model::default();
    let map = EntityMap { nodes: vec![], elements: vec![] };
    let artifacts = REQUIRED_ARTIFACT_ROLES.iter().map(|k| (k.to_string(), artifact_digest(k.as_bytes()))).collect();
    let binding = Binding::freeze(&m, &Geometry::default(), &Run::default(), &map, artifacts, BACKEND_VERSION).unwrap();
    let manifest = br#"{"scope":"IO-only bytes, not solver admission"}"#.to_vec();
    let hash = sha256_hex(&manifest);
    let p = ArchivedChunkedTrajectoryParts {
        archive_key: "chunked".into(),
        binding,
        entity_map: map,
        result_id: format!("fieldcraft-result/{hash}"),
        payload_sha256: hash,
        manifest_json: manifest,
        admission_json: br#"{"solve":"failed","valid_prefix":11605,"first_failed":11606,"native_exit":2}"#.to_vec(),
        context_json: b"{}".to_vec(),
        blobs: vec![BlobDescriptor { index: 0, role: "raw_source".into(), bytes: bytes.len() as u64, sha256: sha256_hex(&bytes) }],
        source: Arc::new(Bytes(bytes)),
    };
    (m, p)
}
fn write(path: &std::path::Path, m: &Model, a: &ArchivedChunkedTrajectory) {
    let e = Extras {
        results: vec![(a.archive_key(), StoredRef { definition: a.definition().unwrap(), result: SolvedRef::ExternalChunked(a) })],
        ..Default::default()
    };
    write_with_to(std::fs::File::create(path).unwrap(), m, &Geometry::default(), &[], &Default::default(), &Default::default(), &e).unwrap();
}
#[test]
fn streamed_roundtrip_owned_handle_and_partial_state() {
    let (m, p) = sample(vec![7; 131073]);
    let a = ArchivedChunkedTrajectory::new(p).unwrap();
    let d = std::env::temp_dir().join(format!("external-chunked-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    let path = d.join("first.fieldcraft");
    write(&path, &m, &a);
    let original = std::fs::read(&path).unwrap();
    let f = read_path_streamed(&path).unwrap();
    std::fs::rename(&path, d.join("moved.fieldcraft")).unwrap();
    let Solved::ExternalChunked(b) = &f.results[0].result else { panic!("wrong kind") };
    assert_eq!(a, b.clone());
    write(&path, &f.model, b);
    assert_eq!(original, std::fs::read(&path).unwrap());
    assert!(String::from_utf8_lossy(b.admission_json()).contains("11606"));
}
#[test]
fn changed_truncated_and_extra_blob_refused() {
    for bytes in [vec![1; 20], vec![7; 19], vec![7; 21]] {
        let (m, mut p) = sample(vec![7; 20]);
        p.source = Arc::new(Bytes(bytes));
        let a = ArchivedChunkedTrajectory::new(p).unwrap();
        let e = Extras {
            results: vec![(a.archive_key(), StoredRef { definition: a.definition().unwrap(), result: SolvedRef::ExternalChunked(&a) })],
            ..Default::default()
        };
        assert!(write_with_to(Cursor::new(Vec::new()), &m, &Geometry::default(), &[], &Default::default(), &Default::default(), &e).is_err());
    }
}
#[test]
fn descriptors_limits_and_identity_refused() {
    let (_, mut p) = sample(vec![1]);
    p.blobs[0].index = 1;
    assert!(ArchivedChunkedTrajectory::new(p).unwrap_err().to_string().contains("DuplicateBlob"));
    let (_, mut p) = sample(vec![1]);
    p.blobs[0].bytes = 256 * 1024 * 1024 + 1;
    assert!(ArchivedChunkedTrajectory::new(p).unwrap_err().to_string().contains("Limit"));
    let (_, mut p) = sample(vec![1]);
    p.payload_sha256 = "0".repeat(64);
    assert!(ArchivedChunkedTrajectory::new(p).is_err());
    let (mut m, p) = sample(vec![1]);
    let a = ArchivedChunkedTrajectory::new(p).unwrap();
    m.set_node(1, [0., 0., 0.]).unwrap();
    assert!(a.validate_binding(&m, &Geometry::default(), &Run::default()).is_err());
}
#[test]
fn unknown_schema_named_refusal() {
    let (m, p) = sample(vec![1]);
    let a = ArchivedChunkedTrajectory::new(p).unwrap();
    let mut data =
        results::encode(&[(a.archive_key(), StoredRef { definition: a.definition().unwrap(), result: SolvedRef::ExternalChunked(&a) })]).unwrap();
    let needle = CHUNKED_SCHEMA.as_bytes();
    let i = data.windows(needle.len()).position(|x| x == needle).unwrap();
    data[i + needle.len() - 1] = b'9';
    assert!(results::decode(&data).unwrap_err().to_string().contains("ExternalArchiveSchemaUnsupported"));
    drop(m);
}

#[test]
fn bounded_copy_releases_source_and_checks_metadata_limits() {
    use std::sync::atomic::{AtomicBool, Ordering};
    struct Source {
        active: Arc<AtomicBool>,
        data: Vec<u8>,
    }
    struct Guard<'a> {
        active: Arc<AtomicBool>,
        bytes: Cursor<&'a [u8]>,
    }
    impl Drop for Guard<'_> {
        fn drop(&mut self) {
            self.active.store(false, Ordering::SeqCst);
        }
    }
    impl Read for Guard<'_> {
        fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
            assert!(b.len() <= COPY_BUFFER_BYTES);
            self.bytes.read(b)
        }
    }
    impl BlobSource for Source {
        fn open(&self, _: u32) -> fieldcraft_io::Result<Box<dyn Read + '_>> {
            assert!(!self.active.swap(true, Ordering::SeqCst));
            Ok(Box::new(Guard { active: self.active.clone(), bytes: Cursor::new(&self.data) }))
        }
    }
    let active = Arc::new(AtomicBool::new(false));
    let (m, mut p) = sample(vec![3; 131073]);
    p.source = Arc::new(Source { active: active.clone(), data: vec![3; 131073] });
    let a = ArchivedChunkedTrajectory::new(p).unwrap();
    let e = Extras {
        results: vec![(a.archive_key(), StoredRef { definition: a.definition().unwrap(), result: SolvedRef::ExternalChunked(&a) })],
        ..Default::default()
    };
    write_with_to(Cursor::new(Vec::new()), &m, &Geometry::default(), &[], &Default::default(), &Default::default(), &e).unwrap();
    assert!(!active.load(Ordering::SeqCst));
    let (_, mut p) = sample(vec![]);
    p.admission_json = vec![b' '; 1024 * 1024 + 1];
    assert!(ArchivedChunkedTrajectory::new(p).unwrap_err().to_string().contains("ExternalArchiveLimit"));
}

#[test]
fn project_missing_changed_and_unlisted_members_are_rejected() {
    use std::io::Write;
    let (m, p) = sample(vec![7; 10]);
    let a = ArchivedChunkedTrajectory::new(p).unwrap();
    let dir = std::env::temp_dir().join(format!("chunk-hostile-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let original = dir.join("original.fieldcraft");
    write(&original, &m, &a);
    for mode in ["missing", "changed", "extra"] {
        let mut z = zip::ZipArchive::new(std::fs::File::open(&original).unwrap()).unwrap();
        let path = dir.join(format!("{mode}.fieldcraft"));
        let mut w = zip::ZipWriter::new(std::fs::File::create(&path).unwrap());
        for i in 0..z.len() {
            let mut f = z.by_index(i).unwrap();
            let name = f.name().unwrap().to_string();
            if mode == "missing" && name.starts_with("external-") {
                continue;
            }
            let mut data = Vec::new();
            f.read_to_end(&mut data).unwrap();
            if mode == "changed" && name.starts_with("external-") {
                data[0] ^= 1
            }
            w.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
            w.write_all(&data).unwrap();
        }
        if mode == "extra" {
            w.start_file(format!("external-{}.bin", "0".repeat(64)), zip::write::SimpleFileOptions::default()).unwrap();
            w.write_all(b"x").unwrap();
        }
        w.finish().unwrap();
        assert!(read_path_streamed(&path).is_err());
    }
}
