//! Protocol fixtures only. Solver labels exercise gates, not numerical qualification.
mod external_support;
use external_support::Fixture;
use fieldcraft_engine::{
    Session,
    external::{self, chunked::*, chunked_archive::*, context::*, receive::*, run::*, *},
};
use serde_json::json;
use std::{io::Read, path::Path};
fn file(root: &Path, name: &str, role: &str, bytes: &[u8]) -> ChunkFile {
    std::fs::write(root.join(name), bytes).unwrap();
    ChunkFile { locator: name.into(), role: role.into(), sha256: fieldcraft_io::sha256_hex(bytes), bytes: bytes.len() as u64 }
}
fn package(f: &Fixture, failed: bool) -> (Manifest, Request) {
    let mut h = f.envelope.clone();
    h.frames.clear();
    h.terminal = None;
    h.goal = RunGoal { last_frame: 3, end_time_s: 3.0 };
    let mut value = serde_json::to_value(&h).unwrap();
    let map = value.as_object_mut().unwrap();
    map.remove("frames");
    map.remove("terminal");
    map.insert("schema".into(), "fieldcraft-external-header/2".into());
    let header = file(&f.root, "header.json", "header", &serde_json::to_vec(&value).unwrap());
    let mut frames = vec![];
    let mut bytes = vec![];
    for i in 1..=3 {
        let mut facts = f.envelope.frames[0].facts.clone();
        facts.frame = i;
        facts.time_s = i as f64;
        if failed && i == 2 {
            facts.hit_newton_limit = Observed::Known { value: true, source: h.source_scene.clone() };
            facts.converged = Observed::Known { value: false, source: h.source_scene.clone() };
        }
        let fields = f.envelope.frames[0]
            .fields
            .iter()
            .map(|f| {
                let data = match &f.data {
                    FieldData::Present { bytes: b, sha256, available, source } => {
                        let at = bytes.len() as u64;
                        bytes.extend_from_slice(b);
                        PackedData::Present {
                            offset: at,
                            bytes: b.len() as u64,
                            sha256: sha256.clone(),
                            mask: available.clone(),
                            source: source.clone(),
                        }
                    }
                    FieldData::Unavailable { reason } => PackedData::Unavailable { reason: reason.clone() },
                };
                PackedField { spec: f.spec.clone(), data }
            })
            .collect();
        frames.push(PackedFrame { facts, fields });
    }
    let metadata =
        file(&f.root, "chunk.json", "metadata", &serde_json::to_vec(&ChunkMetadata { schema: CHUNK_SCHEMA.into(), index: 0, frames }).unwrap());
    let binary = file(&f.root, "chunk.bin", "binary", &bytes);
    let mut sources = f.manifest.sources.clone();
    sources.sort_by(|a, b| a.locator.cmp(&b.locator));
    let manifest = Manifest {
        schema: external::chunked_archive::PACKAGE_SCHEMA.into(),
        header,
        chunks: vec![ChunkSpec { index: 0, first_frame: 1, last_frame: 3, metadata, binary }],
        sources,
        terminal: Some(TerminalClaim {
            outcome: if failed { SolveOutcome::Failed } else { SolveOutcome::Succeeded },
            source: h.source_scene.clone(),
        }),
    };
    let mut expected = f.expected.clone();
    expected.profile.goal = h.goal;
    let request = Request {
        schema: REQUEST_SCHEMA.into(),
        expected,
        acceptance: if failed { Acceptance::PreserveQualifiedPrefix } else { Acceptance::CompleteOnly },
    };
    std::fs::write(f.root.join("incoming-v2.json"), serde_json::to_vec(&manifest).unwrap()).unwrap();
    std::fs::write(f.root.join("expected-v2.json"), serde_json::to_vec(&request).unwrap()).unwrap();
    (manifest, request)
}
fn receiver() -> ReceiverIdentity {
    ReceiverIdentity { source_commit: None, program: None, unknown_reason: Some("protocol fixture receiver".into()) }
}
#[test]
fn partial_admission_is_explicit_and_never_claims_solver_success() {
    let f = Fixture::new("archive-v2-prefix");
    let (m, mut r) = package(&f, true);
    r.acceptance = Acceptance::CompleteOnly;
    let read = |s: &ChunkFile| -> Result<Box<dyn Read>, ContractError> { Ok(Box::new(std::fs::File::open(f.root.join(&s.locator)).unwrap())) };
    let store = f.root.join("store");
    assert!(receive("partial".into(), f.model.clone(), r.clone(), serde_json::to_vec(&m).unwrap(), receiver(), &store, read).is_err());
    assert_eq!(std::fs::read_dir(&store).unwrap().count(), 0);
    r.acceptance = Acceptance::PreserveQualifiedPrefix;
    let record = receive("partial".into(), f.model.clone(), r, serde_json::to_vec(&m).unwrap(), receiver(), &store, read).unwrap();
    let context = reopen(&record, f.model.clone()).unwrap();
    let a: Admission = serde_json::from_slice(record.admission_json()).unwrap();
    assert_eq!(a.coverage, Coverage::QualifiedPrefix { last: 1, requested_last: 3 });
    assert_eq!(a.first_rejected_frame, Some(2));
    assert_eq!(a.solve, SolveOutcome::Failed);
    assert_eq!(a.commit, CommitState::NotAttempted);
    assert_eq!(context.publication.state, CommittedOnly::Committed);
    assert_eq!(context.runtime_qualification.state, UnverifiedOnly::Unverified);
    assert_eq!(context.summary.observed_frames, 3);
}
#[test]
fn formal_v2_partial_project_is_self_contained_and_resaves_identically() {
    let f = Fixture::new("archive-v2-command");
    package(&f, true);
    let mut s = Session::default();
    s.execute("file.open", &json!({"path":f.root.join("input.fieldcraft")})).unwrap();
    let result = s
        .execute(
            "results.external.receive",
            &json!({"name":"partial","path":f.root.join("incoming-v2.json"),"profile":f.root.join("expected-v2.json"),"store":f.root.join("store")}),
        )
        .unwrap();
    assert_eq!(result["admission"]["solve"], "failed");
    assert_eq!(result["admission"]["first_rejected_frame"], 2);
    let target = f.root.join("received.fieldcraft");
    s.execute("file.save_as", &json!({"path":target})).unwrap();
    let original = std::fs::read(&target).unwrap();
    drop(s);
    std::fs::rename(f.root.join("source.json"), f.root.join("source-unavailable")).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":target})).unwrap();
    let list = reopened.execute("results.external.list", &json!({})).unwrap();
    assert_eq!(list[0]["admission"]["solve"], "failed");
    assert_eq!(list[0]["qualified_coverage"]["last"], 1);
    reopened.execute("file.save", &json!({})).unwrap();
    assert_eq!(std::fs::read(&target).unwrap(), original);
    assert_eq!(std::fs::read(fieldcraft_io::fs::backup_path(&target)).unwrap(), original);
    let second = f.root.join("resaved.fieldcraft");
    reopened.execute("file.save_as", &json!({"path":second})).unwrap();
    assert_eq!(std::fs::read(second).unwrap(), original);
}
#[test]
fn corrupt_source_unknown_prefix_or_wrong_identity_never_publishes_or_installs() {
    for mode in 0..3 {
        let f = Fixture::new(&format!("archive-v2-refuse-{mode}"));
        let (mut m, r) = package(&f, true);
        if mode == 0 {
            std::fs::write(f.root.join("source.json"), b"changed source").unwrap();
        }
        if mode == 1 {
            let mut wire: ChunkMetadata = serde_json::from_slice(&std::fs::read(f.root.join("chunk.json")).unwrap()).unwrap();
            wire.frames[0].facts.converged = Observed::Unknown { reason: "not observed".into() };
            m.chunks[0].metadata = file(&f.root, "chunk.json", "metadata", &serde_json::to_vec(&wire).unwrap());
        }
        let mut model = f.model.clone();
        if mode == 2 {
            model.model.set_node(1, [9.0, 0.0, 0.0]).unwrap();
        }
        let store = f.root.join("store");
        assert!(
            receive("partial".into(), model, r, serde_json::to_vec(&m).unwrap(), receiver(), &store, |s| Ok(Box::new(
                std::fs::File::open(f.root.join(&s.locator)).unwrap()
            )))
            .is_err()
        );
        assert_eq!(std::fs::read_dir(&store).unwrap().count(), 0);
    }
}
