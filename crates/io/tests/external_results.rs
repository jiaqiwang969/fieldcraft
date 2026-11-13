use fieldcraft_io::{external_results::*, identity::*, results::*, *};
use fieldcraft_model::{ElementKind, Model};
use std::collections::BTreeMap;

fn fixture() -> (Model, ArchivedTrajectory) {
    let mut m = Model::default();
    for (i, xyz) in [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].into_iter().enumerate() {
        m.set_node(i as u32 + 1, xyz).unwrap();
    }
    m.set_element(1, ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
    let map =
        EntityMap { nodes: vec![1, 2, 3, 4], elements: vec![MappedElement { target: 1, kind: ElementKind::Tet4, source_nodes: vec![0, 1, 2, 3] }] };
    let artifacts: BTreeMap<_, _> = REQUIRED_ARTIFACT_ROLES.iter().map(|k| (k.to_string(), artifact_digest(k.as_bytes()))).collect();
    let binding = Binding::freeze(&m, &Geometry::default(), &Run::default(), &map, artifacts, BACKEND_VERSION).unwrap();
    // Owned synthetic IO payload, not a receiver-qualified solver envelope.
    let payload = serde_json::to_vec(&serde_json::json!({"run":{"epoch":1},"fields":[{"availability":"present","bytes":1.25f64.to_le_bytes(),"available":[true,false]},{"availability":"unavailable","reason":"not observed"}]})).unwrap();
    let digest = artifact_digest(&payload);
    let a = ArchivedTrajectory::new(ArchivedTrajectoryParts {
        archive_key: "external-run".into(),
        binding,
        entity_map: map,
        result_id: format!("fieldcraft-result/{digest}"),
        payload_sha256: digest,
        envelope_json: payload,
        admission_json: br#"{"commit":"not_attempted"}"#.to_vec(),
        context_json: br#"{"runtime_qualification":"unverified","kappa":"unknown"}"#.to_vec(),
    })
    .unwrap();
    (m, a)
}
fn save(m: &Model, a: &ArchivedTrajectory) -> fieldcraft_io::Result<Vec<u8>> {
    let extras =
        Extras { results: vec![(&a.archive_key, StoredRef { definition: a.definition()?, result: SolvedRef::External(a) })], ..Default::default() };
    write_with(m, &Geometry::default(), &[], &Default::default(), &Default::default(), &extras)
}
#[test]
fn project_disk_reopen_and_canonical_roundtrip_preserve_all_bytes() {
    let (m, a) = fixture();
    let bytes = save(&m, &a).unwrap();
    let mut archive = zip::ZipArchive::new(std::io::Cursor::new(&bytes)).unwrap();
    let mut manifest = String::new();
    use std::io::Read;
    archive.by_name("fieldcraft.json").unwrap().read_to_string(&mut manifest).unwrap();
    let manifest: serde_json::Value = serde_json::from_str(&manifest).unwrap();
    assert!(manifest.get("identity").is_some(), "external projects retain identity metadata");
    assert!(manifest.get("capabilities").is_some(), "external projects retain capability metadata");
    let dir = std::env::var_os("FIELDCRAFT_KIND13_EVIDENCE").map(std::path::PathBuf::from).unwrap_or_else(std::env::temp_dir);
    fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!("external-archive-{}.fieldcraft", std::process::id()));
    fs::write_atomic(&path, &bytes, false).unwrap();
    let opened = read(&fs::read_capped(&path, MAX_FILE_BYTES).unwrap()).unwrap();
    assert_eq!(opened.results.len(), 1);
    let Solved::External(restored) = &opened.results[0].result else { panic!("kind13 missing") };
    assert_eq!(restored, &a);
    assert_eq!(save(&opened.model, restored).unwrap(), bytes);
    assert_eq!(results::MAGIC, b"MCRSLT07");
    println!("kind13 project bytes={} result={} model_revision={}", bytes.len(), a.result_id, a.binding.snapshot.model_revision);
}
#[test]
fn wrong_binding_mapping_and_content_refused() {
    let (m, a) = fixture();
    let mut changed = m.clone();
    changed.set_node(1, [0., 0., 0.1]).unwrap();
    assert!(save(&changed, &a).unwrap_err().to_string().contains("ReferenceMeshMismatch"));
    let mut b = a.clone();
    b.entity_map.nodes.swap(0, 1);
    b.record_sha256 = b.record_digest().unwrap();
    assert!(save(&m, &b).unwrap_err().to_string().contains("ElementConnectivityMismatch"));
    let mut b = a.clone();
    b.binding.snapshot.model_revision = "0".repeat(64);
    b.record_sha256 = b.record_digest().unwrap();
    assert!(save(&m, &b).unwrap_err().to_string().contains("ModelRevisionMismatch"));
    let mut b = a.clone();
    b.envelope_json.push(b' ');
    assert!(b.validate().unwrap_err().to_string().contains("ExternalArchiveDigestMismatch"));
    let mut b = a.clone();
    b.context_json = b"{}".to_vec();
    assert!(b.validate().is_err());
    let mut b = a.clone();
    b.result_id.push('0');
    assert!(b.validate().unwrap_err().to_string().contains("ExternalArchiveResultIdMismatch"));
    let mut b = a.clone();
    b.binding.backend_version = "0.0.30".into();
    assert!(b.validate().unwrap_err().to_string().contains("UnsupportedBackendVersion"));
    // Independent expected producer identity, not a claim that self-hashes authenticate sources.
    let mut b = a.clone();
    b.binding.artifacts.insert("producer".into(), artifact_digest(b"wrong binary"));
    b.record_sha256 = b.record_digest().unwrap();
    assert!(
        b.binding
            .validate(&m, &Geometry::default(), &Run::default(), &b.entity_map, a.binding.artifacts.clone(), BACKEND_VERSION)
            .unwrap_err()
            .to_string()
            .contains("ProducerIdentityMismatch")
    );
}
#[test]
fn result_headers_duplicates_and_hostile_payloads_refused() {
    let (_, a) = fixture();
    let good = StoredRef { definition: a.definition().unwrap(), result: SolvedRef::External(&a) };
    assert!(
        encode(&[(&a.archive_key, StoredRef { definition: [0; 32], ..good })]).unwrap_err().to_string().contains("ExternalArchiveDefinitionMismatch")
    );
    assert!(encode(&[(&a.archive_key, good), (&a.archive_key, good)]).unwrap_err().to_string().contains("ExternalArchiveDuplicateKey"));
    let bytes = encode(&[(&a.archive_key, good)]).unwrap();
    assert!(decode(&bytes[..bytes.len() - 1]).is_err());
    for magic in [b"MCRSLT05", b"MCRSLT06"] {
        let mut old = bytes.clone();
        old[..8].copy_from_slice(magic);
        assert!(decode(&old).is_err());
    }
    let mut b = a.clone();
    b.schema = "future/2".into();
    assert!(b.validate().unwrap_err().to_string().contains("ExternalArchiveSchemaUnsupported"));
    let mut b = a.clone();
    b.context_json = vec![0xff];
    assert!(b.record_digest().is_err());
    let mut b = a.clone();
    b.archive_key = "a".repeat(1025);
    assert!(b.validate().is_err());
    let mut unknown = serde_json::to_value(&a).unwrap();
    unknown["qualified"] = true.into();
    assert!(serde_json::from_value::<ArchivedTrajectory>(unknown).is_err());
}

#[test]
fn reopened_project_rejects_rehashed_foreign_revision_and_mapping() {
    use std::io::{Cursor, Read, Write};
    let (m, a) = fixture();
    let original = save(&m, &a).unwrap();
    for mapping in [false, true] {
        let mut b = a.clone();
        if mapping {
            b.entity_map.nodes.swap(0, 1);
        } else {
            b.binding.snapshot.model_revision = "0".repeat(64);
        }
        b.record_sha256 = b.record_digest().unwrap();
        let part = encode(&[(&b.archive_key, StoredRef { definition: b.definition().unwrap(), result: SolvedRef::External(&b) })]).unwrap();
        let mut src = zip::ZipArchive::new(Cursor::new(&original)).unwrap();
        let mut dst = zip::ZipWriter::new(Cursor::new(Vec::new()));
        for i in 0..src.len() {
            let mut file = src.by_index(i).unwrap();
            let name = file.name().unwrap().to_string();
            let mut data = Vec::new();
            file.read_to_end(&mut data).unwrap();
            if name == "results.bin" {
                data = part.clone();
            }
            if name == "fieldcraft.json" {
                let mut v: serde_json::Value = serde_json::from_slice(&data).unwrap();
                v["parts"]["results.bin"]["bytes"] = serde_json::json!(part.len());
                v["parts"]["results.bin"]["sha256"] = serde_json::json!(artifact_digest(&part));
                data = serde_json::to_vec(&v).unwrap();
            }
            dst.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
            dst.write_all(&data).unwrap();
        }
        let error = read(&dst.finish().unwrap().into_inner()).err().unwrap().to_string();
        assert!(error.contains(if mapping { "ElementConnectivityMismatch" } else { "ModelRevisionMismatch" }), "{error}");
        println!("rehashed project rejected: {error}");
    }
}

#[test]
fn archive_key_is_not_a_fabricated_native_step_name() {
    let (m, mut a) = fixture();
    a.archive_key = "运行 01".into();
    a.record_sha256 = a.record_digest().unwrap();
    let bytes = save(&m, &a).unwrap();
    assert_eq!(read(&bytes).unwrap().results[0].result.step(), "运行 01");
}
