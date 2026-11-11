//! Seal a measured refusal package before product receive.
//!
//! The raw measured run supplies the contact state; this helper does not mutate a
//! sealed archive in place.  `from_payload_unchecked` is intentionally named
//! and limited to refusal evidence because a normal package must pass
//! `from_payload`.

use fieldcraft_io::{
    external_results::{ArchivedStaticBase, ExternalStaticBase},
    identity::{Binding, EntityMap, MappedElement},
};
use fieldcraft_model::{ElementKind, Model};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
struct RawMesh {
    nodes: Vec<[f64; 3]>,
    tets: Vec<[usize; 4]>,
}

#[derive(Deserialize)]
struct RawPackage {
    archive_key: String,
    mesh: RawMesh,
    external_static_base: ExternalStaticBase,
    admission: Value,
    context: Value,
    source: Value,
}

#[derive(Deserialize, Serialize)]
struct Profile {
    schema: String,
    archive_key: String,
    binding: Binding,
    reference_shapes: Vec<Vec<f64>>,
    mode_count: usize,
}

fn model_and_map(mesh: &RawMesh) -> Result<(Model, EntityMap), Box<dyn std::error::Error>> {
    let mut model = Model::default();
    for (index, xyz) in mesh.nodes.iter().enumerate() {
        model.set_node(index as u32 + 1, *xyz)?;
    }
    for (index, nodes) in mesh.tets.iter().enumerate() {
        model.set_element(index as u32 + 1, ElementKind::Tet4, nodes.iter().map(|node| *node as u32 + 1).collect())?;
    }
    let entity_map = EntityMap {
        nodes: (1..=mesh.nodes.len() as u32).collect(),
        elements: mesh
            .tets
            .iter()
            .enumerate()
            .map(|(index, nodes)| MappedElement { target: index as u32 + 1, kind: ElementKind::Tet4, source_nodes: nodes.to_vec() })
            .collect(),
    };
    Ok((model, entity_map))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let profile_path = PathBuf::from(args.next().ok_or("frozen profile")?);
    let measured_path = PathBuf::from(args.next().ok_or("measured raw package")?);
    let out_path = PathBuf::from(args.next().ok_or("output archive")?);
    if args.next().is_some() {
        return Err("too many arguments".into());
    }
    let profile: Profile = serde_json::from_slice(&fs::read(profile_path)?)?;
    let measured: RawPackage = serde_json::from_slice(&fs::read(measured_path)?)?;
    let (model, entity_map) = model_and_map(&measured.mesh)?;
    if profile.schema != "fieldcraft-external-static-base/receive-profile/1" || profile.archive_key != measured.archive_key {
        return Err("frozen profile does not match measured archive key".into());
    }
    profile.binding.validate(
        &model,
        &fieldcraft_io::Geometry::default(),
        &fieldcraft_io::Run::default(),
        &entity_map,
        profile.binding.artifacts.clone(),
        fieldcraft_io::identity::BACKEND_VERSION,
    )?;
    let archive = ArchivedStaticBase::from_payload_unchecked(
        measured.archive_key,
        profile.binding,
        entity_map,
        measured.external_static_base,
        serde_json::to_vec(&measured.admission)?,
        serde_json::to_vec(&measured.context)?,
        serde_json::to_vec(&measured.source)?,
    )?;
    fs::write(out_path, serde_json::to_vec_pretty(&archive)?)?;
    Ok(())
}
