//! Synthetic protocol inputs, including solver labels solely to exercise the store gate.
//! No libuipc run, physical validation or runtime qualification is claimed by this fixture.
#![allow(dead_code)]
use fieldcraft_engine::external::{context::*, package::*, production::*, receive::*, run::*, transaction::*, *};
use serde_json::Value;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
pub struct Fixture {
    pub root: PathBuf,
    pub model: IoModelContext,
    pub expected: ExpectedReceive,
    pub manifest: PackageManifest,
    pub envelope: ResultEnvelope,
}
fn put(root: &Path, name: &str, role: &str, bytes: &[u8]) -> FileSpec {
    std::fs::write(root.join(name), bytes).unwrap();
    FileSpec {
        locator: name.into(),
        role: role.into(),
        sha256: fieldcraft_io::sha256_hex(bytes),
        bytes: bytes.len() as u64,
        availability: RetainedAvailability::Embedded,
    }
}
impl Fixture {
    pub fn new(name: &str) -> Self {
        let root = std::env::temp_dir().join(format!("fieldcraft-external-{name}-{}", std::process::id()));
        std::fs::create_dir(&root).unwrap();
        let run = RunKey { run_id: "fixture".into(), epoch: 1, worker_generation: "fixture".into() };
        let mut m = fieldcraft_model::Model::default();
        let positions = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for (i, p) in positions.into_iter().enumerate() {
            m.set_node(i as u32 + 1, p).unwrap();
        }
        m.set_element(8, fieldcraft_model::ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
        let entities = EntityMap {
            nodes: vec![1, 2, 3, 4],
            elements: vec![MappedElement { target: 8, kind: fieldcraft_model::ElementKind::Tet4, source_nodes: vec![0, 1, 2, 3] }],
            objects: vec![],
        };
        let mut sources = vec![put(&root, "source.json", "original_observation", b"synthetic protocol fixture")];
        let source = SourceRef {
            sha256: sources[0].sha256.clone(),
            locator: "source.json".into(),
            observer: "protocol fixture; no physical observation".into(),
            kind: EvidenceKind::Solver,
        };
        let mut artifacts = BTreeMap::new();
        for role in fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES {
            let f = put(&root, &format!("{role}.bin"), role, role.as_bytes());
            artifacts.insert(role.into(), f.sha256.clone());
            sources.push(f);
        }
        let model = IoModelContext::new(m, fieldcraft_io::Geometry::default(), fieldcraft_io::Run::default(), artifacts.clone());
        let input = fieldcraft_io::write_with(
            &model.model,
            &model.geometry,
            &[],
            &fieldcraft_io::JournalHeader::default(),
            &fieldcraft_io::View::default(),
            &fieldcraft_io::Extras::default(),
        )
        .unwrap();
        sources.push(put(&root, "input.fieldcraft", "original_input", &input));
        sources.push(put(&root, "entities.json", "entity_map", &serde_json::to_vec(&entities).unwrap()));
        let binding = IoSnapshotProvider.freeze(&model, &entities).unwrap();
        let known = |value| Observed::Known { value, source: source.clone() };
        let facts = FrameFacts {
            run: run.clone(),
            frame: 1,
            time_s: 0.01,
            completed: known(true),
            converged: known(true),
            hit_newton_limit: known(false),
            hit_line_search_limit: known(false),
            world_valid: known(true),
            snapshot_complete: known(true),
        };
        let bytes: Vec<_> = positions.into_iter().flatten().flat_map(f64::to_le_bytes).collect();
        let field = FieldBlock {
            spec: FieldSpec {
                quantity: Quantity::Position,
                location: Location::Node,
                unit: "m".into(),
                coordinate_frame: "world_z_up".into(),
                components: 3,
                moment_origin_m: None,
            },
            data: FieldData::Present { sha256: fieldcraft_io::sha256_hex(&bytes), bytes, available: vec![true; 4], source: source.clone() },
        };
        let mut fields = vec![field];
        for (q, loc, unit, n, origin) in [
            (Quantity::Displacement, Location::Node, "m", 3, None),
            (Quantity::Velocity, Location::Node, "m/s", 3, None),
            (Quantity::ObjectForce, Location::Object, "N", 3, None),
            (Quantity::ObjectMoment, Location::Object, "N*m", 3, Some([0.0; 3])),
            (Quantity::ContactEnergy, Location::Global, "J", 1, None),
        ] {
            fields.push(FieldBlock {
                spec: FieldSpec {
                    quantity: q,
                    location: loc,
                    unit: unit.into(),
                    coordinate_frame: "world_z_up".into(),
                    components: n,
                    moment_origin_m: origin,
                },
                data: FieldData::Unavailable { reason: "not supplied by this protocol fixture".into() },
            });
        }
        let goal = RunGoal { last_frame: 1, end_time_s: 0.01 };
        let envelope = ResultEnvelope {
            schema_version: 1,
            binding,
            run: run.clone(),
            entities,
            goal,
            evidence_kind: EvidenceKind::Solver,
            source_scene: source.clone(),
            contact_controls_sha256: None,
            contact_control_record_sha256: None,
            policy_request_digest: None,
            required_fields: vec![Quantity::Position],
            frames: vec![ResultFrame { facts, fields }],
            terminal: Some(TerminalClaim { outcome: SolveOutcome::Succeeded, source: source.clone() }),
        };
        let expected = ExpectedReceive {
            schema: EXPECTED_SCHEMA.into(),
            run,
            profile: ReceiveProfile {
                goal,
                evidence_kind: EvidenceKind::Solver,
                source_scene: source,
                required_fields: vec![Quantity::Position],
                contact_controls: None,
                control_evidence: None,
                require_effective: false,
                policy_request: None,
                require_policy_applied: false,
            },
            artifacts,
        };
        let manifest = PackageManifest {
            schema: PACKAGE_SCHEMA.into(),
            envelope: put(&root, "envelope.json", "envelope", &serde_json::to_vec(&envelope).unwrap()),
            sources,
        };
        let f = Self { root, model, expected, manifest, envelope };
        f.write_manifest();
        f
    }
    pub fn write_manifest(&self) {
        std::fs::write(self.root.join("incoming.json"), serde_json::to_vec(&self.manifest).unwrap()).unwrap();
        std::fs::write(self.root.join("expected.json"), serde_json::to_vec(&self.expected).unwrap()).unwrap();
    }
    pub fn update_envelope(&mut self) {
        self.manifest.envelope = put(&self.root, "envelope.json", "envelope", &serde_json::to_vec(&self.envelope).unwrap());
        self.write_manifest();
    }
    pub fn prepare(&self) -> std::result::Result<PreparedReceive, ContractError> {
        PreparedReceive::read(
            self.model.clone(),
            self.expected.clone(),
            self.manifest.clone(),
            ReceiverIdentity { source_commit: None, program: None, unknown_reason: Some("protocol fixture".into()) },
            |name, max| fieldcraft_io::fs::read_capped(&self.root.join(name), max).map_err(|e| ContractError::Provider(e.to_string())),
        )
    }
    pub fn params(&self) -> Value {
        serde_json::json!({"path":self.root.join("incoming.json"),"profile":self.root.join("expected.json"),"name":"external-fixture","store":self.root.join("store")})
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}
