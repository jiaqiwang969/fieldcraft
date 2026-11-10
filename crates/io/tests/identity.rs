use fieldcraft_io::{Geometry, JournalHeader, Run, View, identity::*};
use fieldcraft_model::{ElementKind, Model};
use std::collections::BTreeMap;

fn fixture() -> (Model, EntityMap) {
    let mut m = Model::default();
    for (i, xyz) in [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].into_iter().enumerate() {
        m.set_node(i as u32 + 1, xyz).unwrap();
    }
    m.set_element(8, ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
    (m, EntityMap { nodes: vec![1, 2, 3, 4], elements: vec![MappedElement { target: 8, kind: ElementKind::Tet4, source_nodes: vec![0, 1, 2, 3] }] })
}
fn artifacts() -> BTreeMap<String, String> {
    ["producer", "adapter", "receiver"].into_iter().map(|r| (r.into(), artifact_digest(r.as_bytes()))).collect()
}
#[test]
fn persistent_snapshot_and_binding_reopen() {
    let (model, map) = fixture();
    let g = Geometry::default();
    let run = Run::default();
    let binding = Binding::freeze(&model, &g, &run, &map, artifacts(), "0.0.29").unwrap();
    let bytes = fieldcraft_io::write(&model, &g, &[], &JournalHeader::default(), &View::default()).unwrap();
    let dir = std::env::var_os("FIELDCRAFT_IDENTITY_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-identity-{}", std::process::id())));
    fieldcraft_io::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("reference.fieldcraft");
    fieldcraft_io::fs::save_model(&path, &bytes).unwrap();
    let reopened_bytes = fieldcraft_io::fs::read_capped(&path, fieldcraft_io::MAX_FILE_BYTES).unwrap();
    assert_eq!(bytes, reopened_bytes);
    let opened = fieldcraft_io::read(&reopened_bytes).unwrap();
    let stored = serde_json::to_vec(&binding).unwrap();
    fieldcraft_io::fs::write_atomic(&dir.join("binding.json"), &stored, false).unwrap();
    let stored = fieldcraft_io::fs::read_capped(&dir.join("binding.json"), 65536).unwrap();
    let restored: Binding = serde_json::from_slice(&stored).unwrap();
    restored.validate(&opened.model, &opened.geometry, &opened.run, &map, artifacts(), "0.0.29").unwrap();
    assert_eq!(binding.snapshot, snapshot(&opened.model, &opened.geometry, &opened.run).unwrap());
    let again = fieldcraft_io::write(&opened.model, &opened.geometry, &[], &JournalHeader::default(), &View::default()).unwrap();
    assert_eq!(bytes, again);
    println!("snapshot={} binding={}", serde_json::to_string(&binding.snapshot).unwrap(), String::from_utf8(stored).unwrap());
}
#[test]
fn same_counts_changed_reference_rejected() {
    let (mut m, map) = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let b = Binding::freeze(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap();
    m.set_node(2, [2., 0., 0.]).unwrap();
    assert!(b.validate(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap_err().to_string().contains("ReferenceMeshMismatch"));
}
#[test]
fn changed_definition_and_forged_revision_rejected() {
    let (mut m, map) = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let b = Binding::freeze(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap();
    let material = serde_json::from_value(serde_json::json!({"elastic":{"type":"isotropic","young":1000.0,"poisson":0.25},"density":1.0})).unwrap();
    m.add_material("solid", material).unwrap();
    assert!(b.validate(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap_err().to_string().contains("ModelRevisionMismatch"));
    let mut forged = snapshot(&m, &g, &r).unwrap();
    forged.model_id = "forged".into();
    assert!(forged.validate(&m, &g, &r).unwrap_err().to_string().contains("ModelIdentityMismatch"));
}
#[test]
fn wrong_mapping_and_permuted_source_identity_rejected() {
    let (m, mut map) = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let b = Binding::freeze(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap();
    map.nodes[1] = 1;
    assert!(map.validate(&m).unwrap_err().to_string().contains("InvalidNodeMapping"));
    map.nodes = vec![2, 1, 3, 4];
    assert!(map.validate(&m).unwrap_err().to_string().contains("ElementConnectivityMismatch"));
    map.elements[0].source_nodes = vec![1, 0, 2, 3];
    map.validate(&m).unwrap();
    assert!(b.validate(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap_err().to_string().contains("EntityMapMismatch"));
    map.elements[0].source_nodes[0] = 99;
    assert!(map.validate(&m).unwrap_err().to_string().contains("ElementConnectivityMismatch"));
}
#[test]
fn wrong_program_and_backend_rejected() {
    let (m, map) = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let b = Binding::freeze(&m, &g, &r, &map, artifacts(), "0.0.29").unwrap();
    let mut changed = artifacts();
    changed.insert("receiver".into(), artifact_digest(b"different artifact"));
    assert!(b.validate(&m, &g, &r, &map, changed, "0.0.29").unwrap_err().to_string().contains("ProducerIdentityMismatch"));
    assert!(Binding::freeze(&m, &g, &r, &map, artifacts(), "0.0.28").unwrap_err().to_string().contains("UnsupportedBackendVersion"));
    assert!(Binding::freeze(&m, &g, &r, &map, BTreeMap::new(), "0.0.29").is_err());
}
#[test]
fn archive_manifest_identity_is_checked_not_trusted() {
    use std::io::{Cursor, Read, Write};
    let (m, _) = fixture();
    let bytes = fieldcraft_io::write(&m, &Geometry::default(), &[], &JournalHeader::default(), &View::default()).unwrap();
    let mut src = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    let mut dst = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for i in 0..src.len() {
        let mut file = src.by_index(i).unwrap();
        let name = file.name().unwrap().to_string();
        let mut data = Vec::new();
        file.read_to_end(&mut data).unwrap();
        if name == "fieldcraft.json" {
            let mut j: serde_json::Value = serde_json::from_slice(&data).unwrap();
            j["identity"]["model_revision"] = "forged".into();
            data = serde_json::to_vec(&j).unwrap();
        }
        dst.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
        dst.write_all(&data).unwrap();
    }
    let altered = dst.finish().unwrap().into_inner();
    assert!(fieldcraft_io::read(&altered).err().unwrap().to_string().contains("ModelRevisionMismatch"));
}

#[test]
fn geometry_roles_settings_and_unknown_scheme_are_bound() {
    let (m, _) = fixture();
    let source = fieldcraft_io::SourceFile::of("source", b"same source bytes").unwrap();
    let design = Geometry { design: Some(source.clone()), meshes: vec![] };
    let imported = Geometry { design: None, meshes: vec![source] };
    let s = snapshot(&m, &design, &Run::default()).unwrap();
    assert!(s.validate(&m, &imported, &Run::default()).unwrap_err().to_string().contains("ModelRevisionMismatch"));
    let mut changed_run = Run::default();
    changed_run.insert("step".into(), fieldcraft_io::RunSettings { backend: fieldcraft_io::Backend::Cpu, fallback: None });
    assert!(s.validate(&m, &design, &changed_run).unwrap_err().to_string().contains("ModelRevisionMismatch"));
    assert_eq!(artifact_digest(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    let mut unsupported = s.clone();
    unsupported.scheme = "unknown/2".into();
    assert!(unsupported.validate(&m, &design, &Run::default()).unwrap_err().to_string().contains("UnsupportedIdentityScheme"));
}

#[test]
fn legacy_manifest_without_identity_can_be_read_then_upgraded() {
    use std::io::{Cursor, Read, Write};
    let (m, _) = fixture();
    let bytes = fieldcraft_io::write(&m, &Geometry::default(), &[], &JournalHeader::default(), &View::default()).unwrap();
    let mut src = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    let mut dst = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for i in 0..src.len() {
        let mut file = src.by_index(i).unwrap();
        let name = file.name().unwrap().to_string();
        let mut data = Vec::new();
        file.read_to_end(&mut data).unwrap();
        if name == "fieldcraft.json" {
            let mut j: serde_json::Value = serde_json::from_slice(&data).unwrap();
            j.as_object_mut().unwrap().remove("identity");
            data = serde_json::to_vec(&j).unwrap();
        }
        dst.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
        dst.write_all(&data).unwrap();
    }
    let legacy = dst.finish().unwrap().into_inner();
    let opened = fieldcraft_io::read(&legacy).unwrap();
    assert_eq!(snapshot(&m, &Geometry::default(), &Run::default()).unwrap(), snapshot(&opened.model, &opened.geometry, &opened.run).unwrap());
    let upgraded = fieldcraft_io::write(&opened.model, &opened.geometry, &[], &JournalHeader::default(), &View::default()).unwrap();
    let mut zip = zip::ZipArchive::new(Cursor::new(upgraded)).unwrap();
    let mut manifest = String::new();
    zip.by_name("fieldcraft.json").unwrap().read_to_string(&mut manifest).unwrap();
    let value: serde_json::Value = serde_json::from_str(&manifest).unwrap();
    assert_eq!(value["identity"]["scheme"], SCHEME);
}
