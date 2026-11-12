//! CPU protocol tests of shared frame admission. Solver-labelled synthetic fields
//! exercise admission only; these are not native trajectories or runtime qualification.
mod external_support;
use external_support::Fixture;
use fieldcraft_engine::external::{production::*, receive::*, run::*, transaction::SnapshotProvider, *};

struct ModelVerifier(IoModelContext);
impl IdentityVerifier for ModelVerifier {
    fn verify(&self, expected: &ModelBinding, _: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
        IoSnapshotProvider.validate(&self.0, expected, entities)
    }
}
fn header(f: &Fixture, n: u64) -> ResultEnvelope {
    let mut e = f.envelope.clone();
    e.goal = RunGoal { last_frame: n, end_time_s: n as f64 };
    e.frames.clear();
    e.terminal = None;
    e
}
fn frame(f: &Fixture, i: u64) -> ResultFrame {
    let mut x = f.envelope.frames[0].clone();
    x.facts.frame = i;
    x.facts.time_s = i as f64;
    x
}

#[test]
fn chunked_and_legacy_admission_are_identical_for_small_trajectories() {
    let f = Fixture::new("chunked-small");
    let h = header(&f, 240);
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    let frames: Vec<_> = (1..=240).map(|i| frame(&f, i)).collect();
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    for chunk in frames.chunks(17) {
        stream.push_chunk(chunk).unwrap();
    }
    let actual = stream.finish(f.envelope.terminal.as_ref()).unwrap();
    let mut legacy = h.clone();
    legacy.frames = frames;
    legacy.terminal = f.envelope.terminal.clone();
    assert_eq!(actual, receiver.validate(&h.binding, &h.run, &legacy).unwrap());
    assert_eq!(actual.coverage, Coverage::Complete { last: 240 });
}

fn check_long_stream(name: &str, count: u64) {
    let mut f = Fixture::new(name);
    for i in 5..=250 {
        f.model.model.set_node(i, [i as f64, 0.0, 0.0]).unwrap();
    }
    let mut h = header(&f, count);
    h.entities.nodes = (1..=250).collect();
    h.binding = IoSnapshotProvider.freeze(&f.model, &h.entities).unwrap();
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    let bytes: Vec<_> = (0..750).flat_map(|i| (i as f64).to_le_bytes()).collect();
    let digest = fieldcraft_io::sha256_hex(&bytes);
    let mut total = 0;
    for start in (1..=count).step_by(128) {
        let chunk: Vec<_> = (start..=(start + 127).min(count))
            .map(|i| {
                let mut x = frame(&f, i);
                if let FieldData::Present { bytes: b, sha256, available, .. } = &mut x.fields[0].data {
                    *b = bytes.clone();
                    *sha256 = digest.clone();
                    *available = vec![true; 250];
                    total += b.len() + available.len();
                }
                x
            })
            .collect();
        stream.push_chunk(&chunk).unwrap();
    }
    assert!(total > 64 * 1024 * 1024);
    let admission = stream.finish(f.envelope.terminal.as_ref()).unwrap();
    assert_eq!(admission.coverage, Coverage::Complete { last: count });
    assert_eq!(admission.solve, SolveOutcome::Succeeded);
    assert_eq!(admission.commit, CommitState::NotAttempted);
}

#[test]
fn long_stream_crosses_old_aggregate_limit_without_retaining_prior_arrays() {
    check_long_stream("chunked-long", 12400);
}

#[test]
fn double_length_stream_keeps_the_same_chunk_budget() {
    check_long_stream("chunked-double", 24800);
}

#[test]
fn first_failure_gap_unknown_and_time_axis_survive_chunk_boundaries() {
    let f = Fixture::new("chunked-boundary");
    let h = header(&f, 3);
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    for mode in 0..4 {
        let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
        stream.push_chunk(&[frame(&f, 1)]).unwrap();
        let mut failed = frame(&f, 2);
        match mode {
            0 => failed.facts.hit_newton_limit = Observed::Known { value: true, source: h.source_scene.clone() },
            1 => failed.facts.converged = Observed::Unknown { reason: "not observed".into() },
            2 => failed.facts.time_s = 0.5,
            _ => failed.fields.pop().map(|_| ()).unwrap(),
        }
        stream.push_chunk(&[failed, frame(&f, 3)]).unwrap();
        let a = stream.finish(f.envelope.terminal.as_ref()).unwrap();
        assert_eq!(a.coverage, Coverage::QualifiedPrefix { last: 1, requested_last: 3 });
        assert_eq!(a.first_rejected_frame, Some(2));
        assert_ne!(a.solve, SolveOutcome::Succeeded);
    }
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    stream.push_chunk(&[frame(&f, 1)]).unwrap();
    stream.push_chunk(&[frame(&f, 3)]).unwrap();
    assert_eq!(stream.finish(f.envelope.terminal.as_ref()).unwrap().first_rejected_frame, Some(2));
}

#[test]
fn structural_failure_poisoning_units_digest_epoch_and_oversize_are_refused() {
    let f = Fixture::new("chunked-bad");
    let h = header(&f, 1);
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    for mode in 0..4 {
        let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
        let mut x = frame(&f, 1);
        match mode {
            0 => x.fields[0].spec.unit = "mm".into(),
            1 => {
                if let FieldData::Present { sha256, .. } = &mut x.fields[0].data {
                    *sha256 = "0".repeat(64);
                }
            }
            2 => x.facts.run.epoch += 1,
            _ => {
                if let FieldData::Present { bytes, .. } = &mut x.fields[0].data {
                    *bytes = vec![0; 64 * 1024 * 1024 + 1];
                }
            }
        }
        let err = stream.push_chunk(&[x]).unwrap_err();
        if mode == 3 {
            assert_eq!(err, ContractError::Invalid("ExternalFieldLimit: bytes and mask"));
        }
        assert!(stream.push_chunk(&[frame(&f, 1)]).is_err());
        assert!(stream.finish(f.envelope.terminal.as_ref()).is_err());
    }
}

#[test]
fn policy_qualification_and_missing_terminal_cannot_be_upgraded_by_chunking() {
    let f = Fixture::new("chunked-policy");
    let mut h = header(&f, 1);
    h.policy_request_digest = Some("a".repeat(64));
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    stream.push_chunk(&[frame(&f, 1)]).unwrap();
    let a = stream.finish(f.envelope.terminal.as_ref()).unwrap();
    assert_eq!(a.solve, SolveOutcome::Unknown);
    assert_eq!(a.policy.unwrap().runtime_qualification, EvidenceCheck::Unverified);
    h.policy_request_digest = None;
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    stream.push_chunk(&[frame(&f, 1)]).unwrap();
    let a = stream.finish(None).unwrap();
    assert!(!a.sealed);
    assert_ne!(a.solve, SolveOutcome::Succeeded);
}

fn packed(f: &Fixture) -> (fieldcraft_engine::external::chunked::ChunkSpec, Vec<u8>, Vec<u8>) {
    use fieldcraft_engine::external::chunked::*;
    let source = frame(f, 1);
    let mut binary = Vec::new();
    let fields = source
        .fields
        .into_iter()
        .map(|field| {
            let data = match field.data {
                FieldData::Present { bytes, sha256, available, source } => {
                    let offset = binary.len() as u64;
                    binary.extend_from_slice(&bytes);
                    PackedData::Present { offset, bytes: bytes.len() as u64, sha256, mask: available, source }
                }
                FieldData::Unavailable { reason } => PackedData::Unavailable { reason },
            };
            PackedField { spec: field.spec, data }
        })
        .collect();
    let metadata =
        serde_json::to_vec(&ChunkMetadata { schema: CHUNK_SCHEMA.into(), index: 0, frames: vec![PackedFrame { facts: source.facts, fields }] })
            .unwrap();
    let file = |locator: &str, bytes: &[u8]| ChunkFile {
        locator: locator.into(),
        role: "chunk".into(),
        sha256: fieldcraft_io::sha256_hex(bytes),
        bytes: bytes.len() as u64,
    };
    let spec = ChunkSpec { index: 0, first_frame: 1, last_frame: 1, metadata: file("chunk.json", &metadata), binary: file("chunk.bin", &binary) };
    (spec, metadata, binary)
}

#[test]
fn wire_chunk_exact_bytes_and_semantic_admission_share_original_rules() {
    use fieldcraft_engine::external::chunked::*;
    let f = Fixture::new("chunk-wire");
    let h = header(&f, 1);
    let (spec, metadata, binary) = packed(&f);
    let frames = read_chunk(0, &spec, &h.entities, |file| {
        Ok(Box::new(std::io::Cursor::new(if file.locator == "chunk.json" { metadata.clone() } else { binary.clone() })))
    })
    .unwrap();
    assert_eq!(frames, vec![frame(&f, 1)]);
    let receiver = ExternalReceiver::new(ModelVerifier(f.model.clone()));
    let mut stream = receiver.begin_stream(&h.binding, &h.run, &h).unwrap();
    stream.push_chunk(&frames).unwrap();
    assert_eq!(stream.finish(f.envelope.terminal.as_ref()).unwrap().coverage, Coverage::Complete { last: 1 });
}

#[test]
fn oversize_field_metadata_and_path_are_rejected_before_binary_open() {
    use fieldcraft_engine::external::chunked::*;
    let f = Fixture::new("chunk-wire-over");
    for mode in 0..3 {
        let (mut spec, metadata, _) = packed(&f);
        let mut value: serde_json::Value = serde_json::from_slice(&metadata).unwrap();
        if mode == 0 {
            value["frames"][0]["fields"][0]["data"]["bytes"] = (BINARY_CAP + 1).into();
        }
        let metadata = serde_json::to_vec(&value).unwrap();
        spec.metadata.bytes = metadata.len() as u64;
        spec.metadata.sha256 = fieldcraft_io::sha256_hex(&metadata);
        if mode == 1 {
            spec.metadata.bytes = METADATA_CAP + 1;
        }
        if mode == 2 {
            spec.metadata.locator = "../escape".into();
        }
        let mut opened = Vec::new();
        let error = read_chunk(0, &spec, &f.envelope.entities, |file| {
            opened.push(file.locator.clone());
            Ok(Box::new(std::io::Cursor::new(metadata.clone())))
        })
        .unwrap_err();
        assert!(!opened.iter().any(|p| p == "chunk.bin"));
        if mode == 0 {
            assert_eq!(error, ContractError::Invalid("ExternalFieldLimit: bytes and mask"));
        } else {
            assert!(opened.is_empty());
        }
    }
}

#[test]
fn missing_extra_changed_bytes_and_nonfinite_values_are_rejected() {
    use fieldcraft_engine::external::chunked::*;
    let f = Fixture::new("chunk-wire-integrity");
    for mode in 0..4 {
        let (mut spec, mut metadata, mut binary) = packed(&f);
        match mode {
            0 => {
                binary.pop();
            }
            1 => binary.push(0),
            2 => binary[0] ^= 1,
            _ => {
                binary[..8].copy_from_slice(&f64::NAN.to_le_bytes());
                let sha = fieldcraft_io::sha256_hex(&binary);
                spec.binary.sha256 = sha.clone();
                let mut value: serde_json::Value = serde_json::from_slice(&metadata).unwrap();
                value["frames"][0]["fields"][0]["data"]["sha256"] = sha.into();
                metadata = serde_json::to_vec(&value).unwrap();
                spec.metadata.bytes = metadata.len() as u64;
                spec.metadata.sha256 = fieldcraft_io::sha256_hex(&metadata);
            }
        }
        let error = read_chunk(0, &spec, &f.envelope.entities, |file| {
            Ok(Box::new(std::io::Cursor::new(if file.locator == "chunk.json" { metadata.clone() } else { binary.clone() })))
        })
        .unwrap_err();
        if mode == 2 {
            assert_eq!(error, ContractError::ChecksumMismatch);
        }
        if mode == 3 {
            assert_eq!(error, ContractError::Invalid("nonfinite field value"));
        }
    }
}

#[test]
fn chunk_order_and_unreferenced_binary_ranges_are_rejected() {
    use fieldcraft_engine::external::chunked::*;
    let f = Fixture::new("chunk-wire-order");
    for mode in 0..3 {
        let (mut spec, metadata, binary) = packed(&f);
        if mode == 0 {
            spec.index = 1;
        }
        let mut value: serde_json::Value = serde_json::from_slice(&metadata).unwrap();
        if mode == 1 {
            value["frames"][0]["fields"][0]["data"]["offset"] = 1.into();
        }
        if mode == 2 {
            spec.binary.bytes += 1;
        }
        let metadata = serde_json::to_vec(&value).unwrap();
        spec.metadata.bytes = metadata.len() as u64;
        spec.metadata.sha256 = fieldcraft_io::sha256_hex(&metadata);
        let mut opens = 0;
        assert!(
            read_chunk(0, &spec, &f.envelope.entities, |file| {
                opens += 1;
                Ok(Box::new(std::io::Cursor::new(if file.locator == "chunk.json" { metadata.clone() } else { binary.clone() })))
            })
            .is_err()
        );
        assert!(opens <= 1);
    }
}

#[test]
fn stream_host_rechecks_identity_run_and_frozen_profile_before_reading_chunks() {
    use fieldcraft_engine::external::transaction::ReceiverHost;
    let f = Fixture::new("chunk-host");
    let h = header(&f, 3);
    let mut profile = f.expected.profile.clone();
    profile.goal = h.goal;
    let host = ReceiverHost::new(f.model.clone(), h.run.clone(), IoSnapshotProvider).unwrap();
    let tx = host.begin(&h.run, h.entities.clone(), profile.clone()).unwrap();
    let a = tx
        .validate_stream(&h, f.envelope.terminal.as_ref(), |s| {
            s.push_chunk(&[frame(&f, 1)])?;
            s.push_chunk(&[frame(&f, 2), frame(&f, 3)])
        })
        .unwrap();
    assert_eq!(a.coverage, Coverage::Complete { last: 3 });
    assert_eq!(a.commit, CommitState::NotAttempted);
    let mut wrong = h.clone();
    wrong.required_fields.push(Quantity::Velocity);
    let mut reads = 0;
    assert!(
        tx.validate_stream(&wrong, f.envelope.terminal.as_ref(), |_| {
            reads += 1;
            Ok(())
        })
        .is_err()
    );
    assert_eq!(reads, 0);
    host.update_model(|m| {
        m.model.set_node(1, [0.2, 0.0, 0.0]).unwrap();
    })
    .unwrap();
    assert!(matches!(
        tx.validate_stream(&h, f.envelope.terminal.as_ref(), |_| {
            reads += 1;
            Ok(())
        }),
        Err(ContractError::IdentityMismatch(_))
    ));
    assert_eq!(reads, 0);
    let mut next = h.run.clone();
    next.epoch += 1;
    next.worker_generation = "replacement".into();
    host.replace_run(next).unwrap();
    assert_eq!(
        tx.validate_stream(&h, None, |_| {
            reads += 1;
            Ok(())
        }),
        Err(ContractError::StaleRun)
    );
    assert_eq!(reads, 0);
}

#[test]
fn stream_host_preserves_failure_and_releases_lease_after_reader_failure() {
    use fieldcraft_engine::external::transaction::ReceiverHost;
    let f = Fixture::new("chunk-host-failure");
    let h = header(&f, 3);
    let mut profile = f.expected.profile.clone();
    profile.goal = h.goal;
    let host = ReceiverHost::new(f.model.clone(), h.run.clone(), IoSnapshotProvider).unwrap();
    let tx = host.begin(&h.run, h.entities.clone(), profile.clone()).unwrap();
    let a = tx
        .validate_stream(&h, f.envelope.terminal.as_ref(), |s| {
            s.push_chunk(&[frame(&f, 1)])?;
            let mut failed = frame(&f, 2);
            failed.facts.hit_line_search_limit = Observed::Known { value: true, source: h.source_scene.clone() };
            s.push_chunk(&[failed, frame(&f, 3)])
        })
        .unwrap();
    assert_eq!(a.coverage, Coverage::QualifiedPrefix { last: 1, requested_last: 3 });
    assert_eq!(a.first_rejected_frame, Some(2));
    assert_eq!(a.solve, SolveOutcome::Failed);
    assert!(tx.validate_stream(&h, None, |_| Err(ContractError::Provider("reader interrupted".into()))).is_err());
    drop(tx);
    assert!(host.begin(&h.run, h.entities.clone(), profile).is_ok());
}
