//! Independent IO evidence runner. Does not perform receiver admission or claim solver success.
use fieldcraft_io::{external_results::*, identity::*, results::*, *};
use serde::Deserialize;
use std::{
    io::{Read, Seek, SeekFrom, Write},
    path::PathBuf,
    sync::Arc,
};
#[derive(Deserialize)]
struct FileRange {
    path: PathBuf,
    offset: u64,
    bytes: u64,
    sha256: String,
}
#[derive(Deserialize)]
struct Spec {
    reference: PathBuf,
    ranges: Vec<FileRange>,
    manifest: serde_json::Value,
    admission: serde_json::Value,
    artifacts: std::collections::BTreeMap<String, String>,
}
struct Sources(Vec<FileRange>);
impl BlobSource for Sources {
    fn open(&self, index: u32) -> fieldcraft_io::Result<Box<dyn Read + '_>> {
        let d = self.0.get(index as usize).ok_or_else(|| Error::Invalid("missing source".into()))?;
        let mut f = std::fs::File::open(&d.path).map_err(|e| Error::Io(e.to_string()))?;
        f.seek(SeekFrom::Start(d.offset)).map_err(|e| Error::Io(e.to_string()))?;
        Ok(Box::new(f.take(d.bytes)))
    }
}
fn save(path: &std::path::Path, f: &ModelFile) -> fieldcraft_io::Result<()> {
    let results = f.results.iter().map(|r| (r.result.step(), StoredRef { definition: r.definition, result: r.result.as_ref() })).collect();
    let e = Extras {
        deck_source: f.deck_source.as_ref(),
        results,
        embed: f.embedded.iter().map(|(k, v)| (k.clone(), v.as_slice())).collect(),
        run: f.run.clone(),
    };
    let temporary = path.with_extension(format!("tmp-{}", std::process::id()));
    let file = std::fs::File::create(&temporary).map_err(|e| Error::Io(e.to_string()))?;
    let mut file = write_with_to(file, &f.model, &f.geometry, &f.journal, &f.journal_header, &f.view, &e)?;
    file.flush().map_err(|e| Error::Io(e.to_string()))?;
    file.sync_all().map_err(|e| Error::Io(e.to_string()))?;
    std::fs::rename(temporary, path).map_err(|e| Error::Io(e.to_string()))?;
    Ok(())
}
fn run() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let mode = args.get(1).ok_or("mode")?;
    let input = PathBuf::from(args.get(2).ok_or("input")?);
    if mode == "pack" {
        let s: Spec = serde_json::from_reader(std::fs::File::open(input)?)?;
        let mut f = read_path_streamed(&s.reference)?;
        f.results.clear();
        let nodes: Vec<_> = f.model.nodes().keys().copied().collect();
        let index: std::collections::BTreeMap<_, _> = nodes.iter().enumerate().map(|(i, n)| (*n, i)).collect();
        let elements = f
            .model
            .elements()
            .iter()
            .map(|(label, e)| MappedElement { target: *label, kind: e.kind, source_nodes: e.nodes.iter().map(|n| index[n]).collect() })
            .collect();
        let map = EntityMap { nodes, elements };
        let artifacts = s.artifacts;
        let binding = Binding::freeze(&f.model, &f.geometry, &f.run, &map, artifacts, BACKEND_VERSION)?;
        let manifest = serde_json::to_vec(&s.manifest)?;
        let hash = sha256_hex(&manifest);
        let blobs = s
            .ranges
            .iter()
            .enumerate()
            .map(|(i, r)| BlobDescriptor { index: i as u32, role: "raw_source_fragment".into(), bytes: r.bytes, sha256: r.sha256.clone() })
            .collect();
        let a = ArchivedChunkedTrajectory::new(ArchivedChunkedTrajectoryParts {
            archive_key: "retained-partial-io".into(),
            binding,
            entity_map: map,
            result_id: format!("fieldcraft-result/{hash}"),
            payload_sha256: hash,
            manifest_json: manifest,
            admission_json: serde_json::to_vec(&s.admission)?,
            context_json: br#"{"scope":"IO provenance fixture only; no production admission or qualification","runtime_qualification":"unverified"}"#
                .to_vec(),
            blobs,
            source: Arc::new(Sources(s.ranges)),
        })?;
        f.results.push(Stored { definition: a.definition()?, result: Solved::ExternalChunked(a) });
        save(std::path::Path::new(args.get(3).ok_or("output")?), &f)?;
    } else {
        let f = read_path_streamed(&input)?;
        for r in &f.results {
            if let Solved::ExternalChunked(a) = &r.result {
                println!("{}", String::from_utf8_lossy(a.admission_json()));
                println!("blobs={} result={}", a.blobs().len(), a.result_id());
            }
        }
        if mode == "resave" {
            save(std::path::Path::new(args.get(3).ok_or("output")?), &f)?;
        }
    }
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1)
    }
}
