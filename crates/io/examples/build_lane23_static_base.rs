//! Package the real lane-23 runtime payload with the same identity machinery
//! used by the product receiver.  This is an evidence builder, not a second
//! acceptance path: the engine command still recomputes the modal package.

use fieldcraft_io::{
    Geometry, Run, View,
    external_results::ArchivedStaticBase,
    identity::{Binding, EntityMap, MappedElement, REQUIRED_ARTIFACT_ROLES, artifact_digest},
    write_with,
};
use fieldcraft_model::{ElementKind, Model};
use serde::Deserialize;
use serde_json::Value;
use std::{collections::BTreeMap, env, fs, path::PathBuf};

#[derive(Debug, Deserialize)]
struct RawMesh {
    nodes: Vec<[f64; 3]>,
    tets: Vec<[usize; 4]>,
}

#[derive(Debug, Deserialize)]
struct RawPackage {
    archive_key: String,
    mesh: RawMesh,
    external_static_base: fieldcraft_io::external_results::ExternalStaticBase,
    reference_shapes: Vec<Vec<f64>>,
    mode_count: usize,
    admission: Value,
    context: Value,
    source: Value,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let raw_path = PathBuf::from(args.next().ok_or("raw package path")?);
    let out_dir = PathBuf::from(args.next().ok_or("output directory")?);
    fs::create_dir_all(&out_dir)?;
    let raw: RawPackage = serde_json::from_slice(&fs::read(&raw_path)?)?;
    if raw.external_static_base.modal.is_some() {
        return Err("raw package must omit producer modal data".into());
    }
    let mut model = Model::default();
    for (index, xyz) in raw.mesh.nodes.iter().enumerate() {
        model.set_node(index as u32 + 1, *xyz)?;
    }
    for (index, nodes) in raw.mesh.tets.iter().enumerate() {
        model.set_element(index as u32 + 1, ElementKind::Tet4, nodes.iter().map(|node| *node as u32 + 1).collect())?;
    }
    let entity_map = EntityMap {
        nodes: (1..=raw.mesh.nodes.len() as u32).collect(),
        elements: raw
            .mesh
            .tets
            .iter()
            .enumerate()
            .map(|(index, nodes)| MappedElement { target: index as u32 + 1, kind: ElementKind::Tet4, source_nodes: nodes.to_vec() })
            .collect(),
    };
    let producer_bytes = serde_json::to_vec(&raw.external_static_base)?;
    let mut artifacts = BTreeMap::new();
    artifacts.insert("producer".into(), artifact_digest(&producer_bytes));
    artifacts.insert("adapter".into(), artifact_digest(b"lane23-product-static-base-entry/v1"));
    artifacts.insert("receiver".into(), artifact_digest(b"fieldcraft-engine-results.external.static_base.receive/v1"));
    debug_assert_eq!(artifacts.len(), REQUIRED_ARTIFACT_ROLES.len());
    let binding = Binding::freeze(&model, &Geometry::default(), &Run::default(), &entity_map, artifacts, fieldcraft_io::identity::BACKEND_VERSION)?;
    let archive = ArchivedStaticBase::from_payload(
        raw.archive_key.clone(),
        binding.clone(),
        entity_map,
        raw.external_static_base,
        serde_json::to_vec(&raw.admission)?,
        serde_json::to_vec(&raw.context)?,
        serde_json::to_vec(&raw.source)?,
    )?;
    fs::write(out_dir.join("real-closed-static-base.json"), serde_json::to_vec_pretty(&archive)?)?;
    let profile = serde_json::json!({
        "schema": "fieldcraft-external-static-base/receive-profile/1",
        "archive_key": raw.archive_key,
        "binding": binding,
        "reference_shapes": raw.reference_shapes,
        "mode_count": raw.mode_count,
    });
    fs::write(out_dir.join("real-static-base-profile.json"), serde_json::to_vec_pretty(&profile)?)?;
    // A real FieldCraft document carrying the same node/element mesh is used by
    // the receive example; it makes the binding check exercise the full map.
    let model_path = out_dir.join("real-static-base-mesh.fieldcraft");
    let bytes = write_with(&model, &Geometry::default(), &[], &Default::default(), &View::default(), &fieldcraft_io::Extras::default())?;
    fs::write(model_path, bytes)?;
    Ok(())
}
