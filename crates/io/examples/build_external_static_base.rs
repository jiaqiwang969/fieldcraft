//! Build the bounded external static-base package.
//!
//! The `expected` invocation is deliberately independent of a producer package:
//! it freezes only the mesh, actual artifact digests and reference modes.  The
//! `package` invocation consumes that frozen profile and a measured raw run.
//! Product receive remains the authority that recomputes modal values.

use fieldcraft_io::{
    Geometry, Run, View,
    external_results::ArchivedStaticBase,
    identity::{Binding, EntityMap, MappedElement, REQUIRED_ARTIFACT_ROLES},
    write_with,
};
use fieldcraft_model::{ElementKind, Model};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{
    collections::BTreeMap,
    env, fs,
    path::{Path, PathBuf},
};

#[derive(Debug, Deserialize, Serialize)]
struct RawMesh {
    nodes: Vec<[f64; 3]>,
    tets: Vec<[usize; 4]>,
}

#[derive(Debug, Deserialize)]
struct RawPackage {
    archive_key: String,
    mesh: RawMesh,
    external_static_base: fieldcraft_io::external_results::ExternalStaticBase,
    admission: Value,
    context: Value,
    source: Value,
}

#[derive(Debug, Deserialize, Serialize)]
struct ReferenceSpec {
    reference_shapes: Vec<Vec<f64>>,
    mode_count: usize,
}

#[derive(Debug, Deserialize, Serialize)]
struct StaticBaseReceiveProfile {
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

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T, Box<dyn std::error::Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn validate_artifacts(artifacts: &BTreeMap<String, String>) -> Result<(), Box<dyn std::error::Error>> {
    if REQUIRED_ARTIFACT_ROLES.iter().any(|role| !artifacts.contains_key(*role)) {
        return Err(format!("artifact map missing one of {:?}", REQUIRED_ARTIFACT_ROLES).into());
    }
    if artifacts.values().any(|digest| !fieldcraft_io::identity::digest_valid(digest)) {
        return Err("artifact map contains a non-SHA256 digest".into());
    }
    Ok(())
}

fn freeze_expected(
    mesh_path: &Path,
    artifacts_path: &Path,
    reference_path: &Path,
    archive_key: String,
    out_path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let mesh: RawMesh = read_json(mesh_path)?;
    let artifacts: BTreeMap<String, String> = read_json(artifacts_path)?;
    validate_artifacts(&artifacts)?;
    let reference: ReferenceSpec = read_json(reference_path)?;
    if reference.mode_count == 0 || reference.reference_shapes.len() < reference.mode_count {
        return Err("reference specification has too few modes".into());
    }
    let (model, entity_map) = model_and_map(&mesh)?;
    let binding = Binding::freeze(&model, &Geometry::default(), &Run::default(), &entity_map, artifacts, fieldcraft_io::identity::BACKEND_VERSION)?;
    let profile = StaticBaseReceiveProfile {
        schema: "fieldcraft-external-static-base/receive-profile/1".into(),
        archive_key,
        binding,
        reference_shapes: reference.reference_shapes,
        mode_count: reference.mode_count,
    };
    fs::write(out_path, serde_json::to_vec_pretty(&profile)?)?;
    Ok(())
}

fn package(raw_path: &Path, profile_path: &Path, out_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let raw: RawPackage = read_json(raw_path)?;
    let profile: StaticBaseReceiveProfile = read_json(profile_path)?;
    if profile.schema != "fieldcraft-external-static-base/receive-profile/1" || profile.archive_key != raw.archive_key {
        return Err("frozen profile does not match raw archive key".into());
    }
    if raw.external_static_base.modal.is_some() {
        return Err("raw package must omit producer modal data".into());
    }
    let (model, entity_map) = model_and_map(&raw.mesh)?;
    profile.binding.validate(
        &model,
        &Geometry::default(),
        &Run::default(),
        &entity_map,
        profile.binding.artifacts.clone(),
        fieldcraft_io::identity::BACKEND_VERSION,
    )?;
    fs::create_dir_all(out_dir)?;
    let archive = ArchivedStaticBase::from_payload(
        raw.archive_key.clone(),
        profile.binding.clone(),
        entity_map,
        raw.external_static_base,
        serde_json::to_vec(&raw.admission)?,
        serde_json::to_vec(&raw.context)?,
        serde_json::to_vec(&raw.source)?,
    )?;
    fs::write(out_dir.join("external-static-base.json"), serde_json::to_vec_pretty(&archive)?)?;
    fs::write(out_dir.join("static-base-profile.json"), serde_json::to_vec_pretty(&profile)?)?;
    let model_path = out_dir.join("external-static-base-mesh.fieldcraft");
    let bytes = write_with(&model, &Geometry::default(), &[], &Default::default(), &View::default(), &fieldcraft_io::Extras::default())?;
    fs::write(model_path, bytes)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    match args.next().as_deref().and_then(|arg| arg.to_str()) {
        Some("expected") => {
            let mesh = PathBuf::from(args.next().ok_or("mesh json")?);
            let artifacts = PathBuf::from(args.next().ok_or("artifacts json")?);
            let reference = PathBuf::from(args.next().ok_or("reference json")?);
            let archive_key = args.next().ok_or("archive key")?.to_string_lossy().into_owned();
            let out = PathBuf::from(args.next().ok_or("profile output")?);
            if args.next().is_some() {
                return Err("expected: too many arguments".into());
            }
            freeze_expected(&mesh, &artifacts, &reference, archive_key, &out)
        }
        Some("package") => {
            let raw = PathBuf::from(args.next().ok_or("raw package json")?);
            let profile = PathBuf::from(args.next().ok_or("frozen profile json")?);
            let out = PathBuf::from(args.next().ok_or("output directory")?);
            if args.next().is_some() {
                return Err("package: too many arguments".into());
            }
            package(&raw, &profile, &out)
        }
        _ => Err("usage: build_external_static_base expected <mesh.json> <artifacts.json> <reference.json> <archive-key> <profile.json> | package <raw.json> <profile.json> <out-dir>".into()),
    }
}
