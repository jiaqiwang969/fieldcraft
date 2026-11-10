//! CPU-only caller for the same package core used by the formal receiving command.
//! This diagnostic caller deliberately does not claim a kind-13 project save.
use fieldcraft_engine::external::context::*;
use fieldcraft_engine::external::package::*;
use fieldcraft_engine::external::production::*;
use fieldcraft_engine::external::{ContractError, EvidenceKind, SourceRef};
use serde_json::{Value, json};
use std::path::{Path, PathBuf};
use std::sync::Arc;
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(p: &Path) -> Result<Vec<u8>> {
    Ok(fieldcraft_io::fs::read_capped(p, MAX_BYTES)?)
}
fn write(p: &Path, v: &impl serde::Serialize) -> Result<()> {
    Ok(fieldcraft_io::fs::write_atomic(p, &serde_json::to_vec_pretty(v)?, false)?)
}
fn model(root: &Path, artifacts: std::collections::BTreeMap<String, String>) -> Result<IoModelContext> {
    let file = fieldcraft_io::read(&read(&root.join("input.fieldcraft"))?)?;
    Ok(IoModelContext::new(file.model, file.geometry, file.run, artifacts))
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let op = args.next().ok_or("operation")?;
    let root = PathBuf::from(args.next().ok_or("root")?).canonicalize()?;
    let before = std::time::Instant::now();
    let result: Value = if op == "prepare" {
        // Conversion of retained source mesh and fields; this does not run a solver.
        let reference: Value = serde_json::from_slice(&read(&root.join("reference-input.json"))?)?;
        let entities: fieldcraft_engine::external::receive::EntityMap = serde_json::from_slice(&read(&root.join("entities.json"))?)?;
        let mut m = fieldcraft_model::Model::default();
        let coords: Vec<[f64; 3]> = serde_json::from_value(reference["nodes"].clone())?;
        if coords.len() != entities.nodes.len() {
            return Err("node count mismatch".into());
        }
        for (label, xyz) in entities.nodes.iter().zip(coords) {
            m.set_node(*label, xyz)?;
        }
        for e in &entities.elements {
            let nodes: Option<Vec<_>> = e.source_nodes.iter().map(|i| entities.nodes.get(*i).copied()).collect();
            m.set_element(e.target, e.kind, nodes.ok_or("connectivity out of bounds")?)?;
        }
        let groups: std::collections::BTreeMap<String, fieldcraft_model::Set> = serde_json::from_value(reference["groups"].clone())?;
        for (name, set) in groups {
            m.add_set(&name, set)?;
        }
        let geometry = fieldcraft_io::Geometry::default();
        let settings = fieldcraft_io::Run::default();
        let input = fieldcraft_io::write_with(
            &m,
            &geometry,
            &[],
            &fieldcraft_io::JournalHeader::default(),
            &fieldcraft_io::View::default(),
            &fieldcraft_io::Extras { run: settings.clone(), ..Default::default() },
        )?;
        fieldcraft_io::fs::write_atomic(&root.join("input.fieldcraft"), &input, false)?;
        let names: std::collections::BTreeMap<String, String> = serde_json::from_slice(&read(&root.join("artifact-files.json"))?)?;
        let artifacts: std::collections::BTreeMap<_, _> =
            names.into_iter().map(|(role, p)| Ok((role, fieldcraft_io::identity::artifact_digest(&read(&root.join(p))?)))).collect::<Result<_>>()?;
        let context = IoModelContext::new(m, geometry, settings, artifacts);
        use fieldcraft_engine::external::transaction::SnapshotProvider;
        let binding = IoSnapshotProvider.freeze(&context, &entities)?;
        let spec: Value = serde_json::from_slice(&read(&root.join("profile.json"))?)?;
        let source: Value = serde_json::from_slice(&read(&root.join("prepared-trajectory.json"))?)?;
        let controls = read(&root.join("controls.json"))?;
        let evidence: fieldcraft_engine::external::control::ControlEvidenceReceipt =
            serde_json::from_slice(&read(&root.join("control-evidence.json"))?)?;
        let profile = fieldcraft_engine::external::transaction::ReceiveProfile {
            goal: serde_json::from_value(spec["goal"].clone())?,
            evidence_kind: EvidenceKind::Solver,
            source_scene: serde_json::from_value(spec["source_scene"].clone())?,
            required_fields: serde_json::from_value(spec["required_fields"].clone())?,
            contact_controls: Some(fieldcraft_engine::external::transaction::ContactControlPayload {
                sha256: fieldcraft_io::sha256_hex(&controls),
                bytes: controls,
            }),
            control_evidence: Some(evidence),
            require_effective: false,
            policy_request: None,
            require_policy_applied: false,
        };
        if spec["require_effective"] != false || spec["require_policy_applied"] != false {
            return Err("this mesh-only converter does not support policy-dependent inputs".into());
        }
        let run = serde_json::from_value(spec["run"].clone())?;
        let envelope = fieldcraft_engine::external::receive::ResultEnvelope {
            schema_version: 1,
            binding,
            run,
            entities,
            goal: profile.goal,
            evidence_kind: EvidenceKind::Solver,
            source_scene: profile.source_scene.clone(),
            required_fields: profile.required_fields.clone(),
            contact_controls_sha256: profile.contact_controls.as_ref().map(|c| c.sha256.clone()),
            contact_control_record_sha256: profile.control_evidence.as_ref().map(|c| c.record_sha256.clone()),
            policy_request_digest: None,
            frames: serde_json::from_value(source["frames"].clone())?,
            terminal: serde_json::from_value(source["terminal"].clone())?,
        };
        let expected = ExpectedReceive { schema: EXPECTED_SCHEMA.into(), run: envelope.run.clone(), profile, artifacts: context.artifacts };
        write(&root.join("input-envelope.json"), &envelope)?;
        write(&root.join("expected.json"), &expected)?;
        json!({"prepared":true,"frames":envelope.frames.len(),"published":false,"run":envelope.run,"runtime_qualification":"unverified"})
    } else if op == "receive" {
        let expected = ExpectedReceive::decode(&read(&root.join("expected.json"))?)?;
        let model = model(&root, expected.artifacts.clone())?;
        let manifest: PackageManifest = serde_json::from_slice(&read(&root.join("incoming.json"))?)?;
        let program = fieldcraft_io::sha256_hex(&read(&std::env::current_exe()?)?);
        let receiver = ReceiverIdentity {
            source_commit: None,
            program: Some(SourceRef {
                sha256: program,
                locator: "current-receiver".into(),
                observer: "executed receiver file".into(),
                kind: EvidenceKind::Solver,
            }),
            unknown_reason: Some("source manifest retained beside this caller; no compile-time commit embedded".into()),
        };
        let prepared = PreparedReceive::read(model, expected, manifest, receiver, |name, max| {
            let p = root.join(name).canonicalize().map_err(|e| ContractError::Provider(e.to_string()))?;
            if !p.starts_with(&root) {
                return Err(ContractError::Invalid("escaped package"));
            }
            fieldcraft_io::fs::read_capped(&p, max).map_err(|e| ContractError::Provider(e.to_string()))
        })?;
        let out = prepared.publish(Arc::new(FileResultStore::new(root.join("published"))?))?;
        write(&root.join("receive-context.json"), &out.context)?;
        json!({"result_id":out.raw.result_id,"admission":out.admission,"publication":out.context.publication,"runtime_qualification":"unverified","kind13_project_saved":false})
    } else if op == "reopen" {
        let context = ReceiveContextV1::decode(&read(&root.join("receive-context.json"))?)?;
        let store = FileResultStore::new(root.join("published"))?;
        let raw = store.read_published(&context.publication.result_id)?;
        let envelope: fieldcraft_engine::external::receive::ResultEnvelope = serde_json::from_slice(&raw.envelope_json)?;
        let (envelope, admission) = context.revalidate(model(&root, envelope.binding.artifacts)?, &raw)?;
        json!({"result_id":raw.result_id,"frames":envelope.frames.len(),"admission":admission,"independent_context_revalidated":true,"new_commit":false,"runtime_qualification":"unverified","kind13_project_saved":false})
    } else {
        return Err("unknown operation".into());
    };
    let result = json!({"result":result,"wall_seconds":before.elapsed().as_secs_f64()});
    println!("{}", serde_json::to_string(&result)?);
    Ok(())
}
