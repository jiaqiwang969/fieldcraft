mod external_support;
use external_support::Fixture;
use fieldcraft_engine::external::{ContractError, Observed, context::*, production::*, receive::*};
use std::sync::Arc;
#[test]
fn original_bytes_context_and_policy_reopen_without_republication() {
    let f = Fixture::new("context-reopen");
    let store = Arc::new(FileResultStore::new(f.root.join("store")).unwrap());
    let p = f.prepare().unwrap().publish(store.clone()).unwrap();
    let id = p.raw.result_id.clone();
    let raw = store.read_published(&id).unwrap();
    assert_eq!(raw.envelope_json, serde_json::to_vec(&f.envelope).unwrap());
    let context = ReceiveContextV1::decode(&serde_json::to_vec(&p.context).unwrap()).unwrap();
    let (e, a) = context.revalidate(f.model.clone(), &raw).unwrap();
    assert_eq!(e, f.envelope);
    assert_eq!(a, p.admission);
    assert_eq!(context.runtime_qualification.state, UnverifiedOnly::Unverified);
    assert_eq!(a.commit, CommitState::NotAttempted);
    assert_eq!(context.publication.state, CommittedOnly::Committed);
    let mut wrong = context.clone();
    wrong.run.epoch += 1;
    assert!(wrong.revalidate(f.model.clone(), &raw).is_err());
    let mut wrong = context.clone();
    wrong.source_inventory.iter_mut().find(|s| s.role == "original_input").unwrap().retained_bytes = None;
    assert!(wrong.revalidate(f.model.clone(), &raw).is_err());
    assert_eq!(std::fs::read_dir(f.root.join("store")).unwrap().count(), 1);
}
#[test]
fn wrong_expected_identity_missing_source_and_failed_frame_never_publish() {
    for case in ["artifact", "source", "frame", "map"] {
        let mut f = Fixture::new(case);
        match case {
            "artifact" => {
                f.expected.artifacts.insert("producer".into(), fieldcraft_io::sha256_hex(b"other"));
            }
            "source" => {
                f.manifest.sources.retain(|s| s.locator != "source.json");
            }
            "frame" => {
                if let Observed::Known { value, .. } = &mut f.envelope.frames[0].facts.converged {
                    *value = false;
                }
                f.update_envelope();
            }
            _ => {
                f.envelope.entities.nodes.swap(0, 1);
                f.update_envelope();
            }
        }
        let store = Arc::new(FileResultStore::new(f.root.join("store")).unwrap());
        assert!(f.prepare().and_then(|p| p.publish(store)).is_err(), "{case}");
        assert_eq!(std::fs::read_dir(f.root.join("store")).unwrap().count(), 0);
    }
}

#[test]
fn payload_corruption_and_replaced_publication_have_distinct_diagnostics() {
    let f = Fixture::new("publication-diagnostics");
    let store = Arc::new(FileResultStore::new(f.root.join("store")).unwrap());
    let published = f.prepare().unwrap().publish(store.clone()).unwrap();
    let original = store.read_published(&published.raw.result_id).unwrap();
    let mut raw = store.read_published(&published.raw.result_id).unwrap();
    raw.envelope_json.push(b' ');
    assert_eq!(published.context.revalidate(f.model.clone(), &raw).unwrap_err(), ContractError::ArchivePayloadMismatch);
    raw.payload_sha256 = fieldcraft_io::sha256_hex(&raw.envelope_json);
    raw.result_id = format!("fieldcraft-result/{}", raw.payload_sha256);
    assert_eq!(published.context.revalidate(f.model.clone(), &raw).unwrap_err(), ContractError::ArchivePublicationMismatch);
    // A changed identifier alone also contradicts the frozen publication.
    let mut raw = store.read_published(&published.raw.result_id).unwrap();
    raw.result_id = format!("fieldcraft-result/{}", fieldcraft_io::sha256_hex(b"another publication"));
    assert_eq!(published.context.revalidate(f.model.clone(), &raw).unwrap_err(), ContractError::ArchivePublicationMismatch);
    let (envelope, admission) = published.context.revalidate(f.model.clone(), &original).unwrap();
    assert_eq!(envelope, f.envelope);
    assert_eq!(admission, published.admission);
    assert_eq!(std::fs::read_dir(f.root.join("store")).unwrap().count(), 1);
}
