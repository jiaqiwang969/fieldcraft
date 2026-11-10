//! Bounded package/2 chunk decoding. The host owns file roots, source retention,
//! the frozen receive profile and atomic publication; this edge never publishes.
use super::context::relative_locator;
use super::receive::{EntityMap, FieldBlock, FieldData, FieldSpec, Location, ResultFrame};
use super::run::FrameFacts;
use super::{ContractError, SourceRef, token_valid};
use serde::{Deserialize, Serialize};
use std::io::Read;

pub const CHUNK_SCHEMA: &str = "fieldcraft-external-chunk/2";
pub const BINARY_CAP: u64 = 64 * 1024 * 1024;
pub const METADATA_CAP: u64 = 1024 * 1024;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkFile {
    pub locator: String,
    pub role: String,
    pub sha256: String,
    pub bytes: u64,
}
impl ChunkFile {
    pub(super) fn validate(&self, cap: u64) -> Result<(), ContractError> {
        if self.bytes > cap {
            return Err(ContractError::Invalid("ExternalChunkLimit: file bytes"));
        }
        if !relative_locator(&self.locator) || !token_valid(&self.role) || !fieldcraft_io::identity::digest_valid(&self.sha256) {
            return Err(ContractError::Invalid("ExternalChunkLayout: file identity"));
        }
        Ok(())
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkSpec {
    pub index: u32,
    pub first_frame: u64,
    pub last_frame: u64,
    pub metadata: ChunkFile,
    pub binary: ChunkFile,
}
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChunkMetadata {
    pub schema: String,
    pub index: u32,
    pub frames: Vec<PackedFrame>,
}
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackedFrame {
    pub facts: FrameFacts,
    pub fields: Vec<PackedField>,
}
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackedField {
    pub spec: FieldSpec,
    pub data: PackedData,
}
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "availability", rename_all = "snake_case", deny_unknown_fields)]
pub enum PackedData {
    Present { offset: u64, bytes: u64, sha256: String, mask: Vec<bool>, source: SourceRef },
    Unavailable { reason: String },
}

pub(super) fn read_exact_file(spec: &ChunkFile, reader: &mut dyn Read) -> Result<Vec<u8>, ContractError> {
    let n = usize::try_from(spec.bytes).map_err(|_| ContractError::Invalid("ExternalChunkLimit: platform size"))?;
    let mut data = vec![0; n];
    reader.read_exact(&mut data).map_err(|e| ContractError::Provider(format!("ExternalChunkRead: {e}")))?;
    let mut extra = [0];
    loop {
        match reader.read(&mut extra) {
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Ok(0) => break,
            Ok(_) => return Err(ContractError::Invalid("ExternalChunkLayout: trailing file bytes")),
            Err(e) => return Err(ContractError::Provider(format!("ExternalChunkRead: {e}"))),
        }
    }
    if fieldcraft_io::sha256_hex(&data) != spec.sha256 {
        return Err(ContractError::ChecksumMismatch);
    }
    Ok(data)
}

/// `open` must apply Roots and package containment to each relative locator. Reads
/// are sized before allocation and consume exact bytes plus one EOF probe. Metadata
/// declarations are checked before opening the binary file. Only one chunk is owned.
pub fn read_chunk<'a>(
    expected_index: u32,
    spec: &ChunkSpec,
    entities: &EntityMap,
    mut open: impl FnMut(&ChunkFile) -> Result<Box<dyn Read + 'a>, ContractError>,
) -> Result<Vec<ResultFrame>, ContractError> {
    if spec.index != expected_index || spec.index >= 8192 {
        return Err(ContractError::Invalid("ExternalChunkOrder: index"));
    }
    let count = spec.last_frame.checked_sub(spec.first_frame).and_then(|n| n.checked_add(1));
    if spec.first_frame == 0 || count.is_none_or(|n| n == 0 || n > 256) {
        return Err(ContractError::Invalid("ExternalChunkLimit: frame range"));
    }
    spec.metadata.validate(METADATA_CAP)?;
    spec.binary.validate(BINARY_CAP)?;
    entities.validate()?;
    let metadata: ChunkMetadata = serde_json::from_slice(&read_exact_file(&spec.metadata, &mut *open(&spec.metadata)?)?)
        .map_err(|e| ContractError::Provider(format!("ExternalChunkMetadata: {e}")))?;
    if metadata.schema != CHUNK_SCHEMA || metadata.index != spec.index || Some(metadata.frames.len() as u64) != count {
        return Err(ContractError::Invalid("ExternalChunkLayout: metadata identity or count"));
    }
    let mut offset = 0u64;
    let mut budget = 0u64;
    for (i, frame) in metadata.frames.iter().enumerate() {
        if Some(frame.facts.frame) != spec.first_frame.checked_add(i as u64) || frame.fields.len() > 7 {
            return Err(ContractError::Invalid("ExternalChunkOrder: frame sequence or fields"));
        }
        for field in &frame.fields {
            field.spec.validate()?;
            if let PackedData::Present { offset: at, bytes, mask, sha256, source } = &field.data {
                let size = bytes.checked_add(mask.len() as u64).ok_or(ContractError::Invalid("ExternalFieldLimit: size overflow"))?;
                if size > BINARY_CAP {
                    return Err(ContractError::Invalid("ExternalFieldLimit: bytes and mask"));
                }
                budget = budget.checked_add(size).ok_or(ContractError::Invalid("ExternalChunkLimit: size overflow"))?;
                if budget > BINARY_CAP {
                    return Err(ContractError::Invalid("ExternalChunkLimit: bytes and masks"));
                }
                let rows = match field.spec.location {
                    Location::Node => entities.nodes.len(),
                    Location::Object => entities.objects.len(),
                    Location::Global => 1,
                };
                let expected = mask.iter().filter(|v| **v).count().checked_mul(field.spec.components).and_then(|n| n.checked_mul(8));
                if mask.len() != rows || expected.map(|n| n as u64) != Some(*bytes) {
                    return Err(ContractError::Invalid("compact array length"));
                }
                if *at != offset || !fieldcraft_io::identity::digest_valid(sha256) {
                    return Err(ContractError::Invalid("ExternalChunkLayout: field range or digest"));
                }
                offset = offset
                    .checked_add(*bytes)
                    .filter(|n| *n <= spec.binary.bytes)
                    .ok_or(ContractError::Invalid("ExternalChunkLayout: binary range"))?;
                source.validate()?;
            }
        }
    }
    if offset != spec.binary.bytes {
        return Err(ContractError::Invalid("ExternalChunkLayout: unreferenced binary bytes"));
    }
    let binary = read_exact_file(&spec.binary, &mut *open(&spec.binary)?)?;
    let mut frames = Vec::with_capacity(metadata.frames.len());
    for frame in metadata.frames {
        let mut fields = Vec::with_capacity(frame.fields.len());
        for field in frame.fields {
            let data = match field.data {
                PackedData::Unavailable { reason } => FieldData::Unavailable { reason },
                PackedData::Present { offset, bytes, sha256, mask, source } => {
                    let start = usize::try_from(offset).map_err(|_| ContractError::Invalid("ExternalChunkLayout: offset"))?;
                    let end = offset
                        .checked_add(bytes)
                        .and_then(|n| usize::try_from(n).ok())
                        .ok_or(ContractError::Invalid("ExternalChunkLayout: offset"))?;
                    let values = binary.get(start..end).ok_or(ContractError::Invalid("ExternalChunkLayout: slice"))?;
                    if fieldcraft_io::sha256_hex(values) != sha256 {
                        return Err(ContractError::ChecksumMismatch);
                    }
                    if values.as_chunks::<8>().0.iter().any(|v| !f64::from_le_bytes(*v).is_finite()) {
                        return Err(ContractError::Invalid("nonfinite field value"));
                    }
                    FieldData::Present { bytes: values.to_vec(), sha256, available: mask, source }
                }
            };
            fields.push(FieldBlock { spec: field.spec, data });
        }
        frames.push(ResultFrame { facts: frame.facts, fields });
    }
    Ok(frames)
}
