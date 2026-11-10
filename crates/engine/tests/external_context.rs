//! Context protocol boundaries only; no native solve or qualification evidence.
use fieldcraft_engine::external::context::*;
use fieldcraft_engine::external::receive::Quantity;
use fieldcraft_engine::external::run::RunGoal;
use fieldcraft_engine::external::transaction::ReceiveProfile;
use fieldcraft_engine::external::*;

fn context() -> ReceiveContextV1 {
    let bytes = b"source".to_vec();
    let sha = fieldcraft_io::sha256_hex(&bytes);
    ReceiveContextV1 {
        schema: SCHEMA.into(),
        run: RunKey { run_id: "protocol".into(), epoch: 1, worker_generation: "worker".into() },
        profile: ReceiveProfile {
            goal: RunGoal { last_frame: 1, end_time_s: 1. },
            evidence_kind: EvidenceKind::ProtocolFixture,
            source_scene: SourceRef {
                sha256: sha.clone(),
                locator: "input.json".into(),
                observer: "fixture".into(),
                kind: EvidenceKind::ProtocolFixture,
            },
            required_fields: vec![Quantity::Position],
            contact_controls: None,
            control_evidence: None,
            require_effective: false,
            policy_request: None,
            require_policy_applied: false,
        },
        publication: PublicationReceipt {
            state: CommittedOnly::Committed,
            result_id: format!("fieldcraft-result/{sha}"),
            payload_sha256: sha.clone(),
        },
        runtime_qualification: UnverifiedQualification { state: UnverifiedOnly::Unverified, reason: "not observed".into() },
        receiver_identity: ReceiverIdentity { source_commit: None, program: None, unknown_reason: Some("fixture".into()) },
        source_inventory: vec![RetainedSource {
            locator: "input.json".into(),
            role: "original_input".into(),
            sha256: sha,
            bytes: bytes.len() as u64,
            availability: RetainedAvailability::Embedded,
            retained_bytes: Some(bytes),
            verification_scope: "fixture bytes".into(),
        }],
    }
}
#[test]
fn frozen_optional_controls_must_be_explicit_and_unknown_cannot_be_promoted() {
    let c = context();
    let raw = serde_json::to_vec(&c).unwrap();
    let read = ReceiveContextV1::decode(&raw).unwrap();
    read.validate().unwrap();
    assert_eq!(c, read);
    for field in ["contact_controls", "control_evidence", "policy_request"] {
        let mut v = serde_json::to_value(&c).unwrap();
        v["profile"].as_object_mut().unwrap().remove(field);
        assert!(ReceiveContextV1::decode(&serde_json::to_vec(&v).unwrap()).is_err(), "{field}");
    }
    let mut v = serde_json::to_value(&c).unwrap();
    v["runtime_qualification"]["state"] = "verified".into();
    assert!(ReceiveContextV1::decode(&serde_json::to_vec(&v).unwrap()).is_err());
}
#[test]
fn retained_source_requires_original_bytes_and_unambiguous_relative_identity() {
    let c = context();
    for which in ["missing", "digest", "size", "escape", "duplicate", "version", "unknown_identity"] {
        let mut v = c.clone();
        match which {
            "missing" => v.source_inventory[0].retained_bytes = None,
            "digest" => v.source_inventory[0].sha256 = fieldcraft_io::sha256_hex(b"changed"),
            "size" => v.source_inventory[0].bytes += 1,
            "escape" => v.source_inventory[0].locator = "../input.json".into(),
            "duplicate" => v.source_inventory.push(v.source_inventory[0].clone()),
            "version" => v.schema = "future/2".into(),
            _ => v.receiver_identity.unknown_reason = None,
        }
        assert!(v.validate().is_err(), "{which}");
    }
}
