//! Versioned, content-addressed receiver snapshots. Not session counters or document lineage.
use crate::{Geometry, Result, Run, invalid, mesh, sha256_hex};
use fieldcraft_model::Model;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SCHEME: &str = "fieldcraft-receiver-snapshot/1";

/// Exact frozen reference family and definition revision, independent of paths and results.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Snapshot {
    pub scheme: String,
    pub model_id: String,
    pub model_revision: String,
    pub reference_mesh_digest: String,
}

fn digest(domain: &str, parts: &[&[u8]]) -> String {
    use sha2::{Digest, Sha256};
    let mut h = Sha256::new();
    h.update(domain.as_bytes());
    for part in parts {
        h.update((part.len() as u64).to_le_bytes());
        h.update(part);
    }
    h.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

pub fn snapshot(model: &Model, geometry: &Geometry, run: &Run) -> Result<Snapshot> {
    let mesh = mesh::encode(model)?;
    let definitions = serde_json::to_vec(&model.without_mesh()).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    let sets = serde_json::to_vec(model.sets()).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    let settings = serde_json::to_vec(run).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    let sources = (geometry.design.as_ref().map(|s| s.sha256()), geometry.meshes.iter().map(|s| s.sha256()).collect::<Vec<_>>());
    let sources = serde_json::to_vec(&sources).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    let reference_mesh_digest = digest("fieldcraft-reference/1", &[&mesh, &sets]);
    Ok(Snapshot {
        scheme: SCHEME.into(),
        model_id: format!("sha256:{reference_mesh_digest}"),
        model_revision: digest(SCHEME, &[reference_mesh_digest.as_bytes(), &definitions, &sources, &settings]),
        reference_mesh_digest,
    })
}

impl Snapshot {
    /// Recompute at both admission and commit; never trust a supplied revision alone.
    pub fn validate(&self, model: &Model, geometry: &Geometry, run: &Run) -> Result<()> {
        if self.scheme != SCHEME {
            return invalid("UnsupportedIdentityScheme");
        }
        let current = snapshot(model, geometry, run)?;
        if self.reference_mesh_digest != current.reference_mesh_digest {
            return invalid("ReferenceMeshMismatch");
        }
        if self.model_id != current.model_id {
            return invalid("ModelIdentityMismatch");
        }
        if self.model_revision != current.model_revision {
            return invalid("ModelRevisionMismatch");
        }
        Ok(())
    }
}

/// Source indices are dense and zero-based. Target labels must cover the entire frozen mesh.
/// The source's ordered connectivity must agree after node mapping; interpolation is excluded.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EntityMap {
    pub nodes: Vec<u32>,
    pub elements: Vec<MappedElement>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappedElement {
    pub target: u32,
    pub kind: fieldcraft_model::ElementKind,
    pub source_nodes: Vec<usize>,
}
impl EntityMap {
    pub fn validate(&self, model: &Model) -> Result<String> {
        if self.nodes.len() != model.nodes().len() || self.elements.len() != model.elements().len() {
            return invalid("EntityCoverageMismatch");
        }
        let nodes: BTreeSet<_> = self.nodes.iter().copied().collect();
        if nodes.len() != self.nodes.len() || nodes.iter().any(|n| model.node(*n).is_none()) {
            return invalid("InvalidNodeMapping");
        }
        let mut targets = BTreeSet::new();
        for source in &self.elements {
            let Some(target) = model.elements().get(&source.target) else {
                return invalid("UnknownElementMapping");
            };
            if !targets.insert(source.target) {
                return invalid("DuplicateElementMapping");
            }
            let mapped: Option<Vec<_>> = source.source_nodes.iter().map(|i| self.nodes.get(*i).copied()).collect();
            if source.kind != target.kind || mapped.as_ref() != Some(&target.nodes) {
                return invalid("ElementConnectivityMismatch");
            }
        }
        let bytes = serde_json::to_vec(self).map_err(|e| crate::Error::Invalid(e.to_string()))?;
        Ok(digest("fieldcraft-entity-map/1", &[&bytes]))
    }
}

/// Frozen lease inputs. Runtime run/epoch/generation remain owned by the runtime manager.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Binding {
    pub snapshot: Snapshot,
    pub entity_map_digest: String,
    pub artifacts: BTreeMap<String, String>,
    pub backend_version: String,
}
impl Binding {
    pub fn freeze(
        model: &Model,
        geometry: &Geometry,
        run: &Run,
        map: &EntityMap,
        artifacts: BTreeMap<String, String>,
        backend_version: &str,
    ) -> Result<Self> {
        if backend_version != "0.0.29" {
            return invalid("UnsupportedBackendVersion");
        }
        if ["producer", "adapter", "receiver"].iter().any(|role| !artifacts.contains_key(*role))
            || artifacts.values().any(|s| s.len() != 64 || !s.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)))
        {
            return invalid("MissingOrInvalidArtifactDigest");
        }
        Ok(Self {
            snapshot: snapshot(model, geometry, run)?,
            entity_map_digest: map.validate(model)?,
            artifacts,
            backend_version: backend_version.into(),
        })
    }
    pub fn validate(
        &self,
        model: &Model,
        geometry: &Geometry,
        run: &Run,
        map: &EntityMap,
        artifacts: BTreeMap<String, String>,
        backend_version: &str,
    ) -> Result<()> {
        self.snapshot.validate(model, geometry, run)?;
        let current = Self::freeze(model, geometry, run, map, artifacts, backend_version)?;
        if self.entity_map_digest != current.entity_map_digest {
            return invalid("EntityMapMismatch");
        }
        if self.backend_version != current.backend_version || self.artifacts != current.artifacts {
            return invalid("ProducerIdentityMismatch");
        }
        Ok(())
    }
}
/// Hash actual artifact bytes; version strings and paths are not byte identities.
pub fn artifact_digest(bytes: &[u8]) -> String {
    sha256_hex(bytes)
}
