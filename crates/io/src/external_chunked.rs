//! Version two external archives: bounded descriptors and owned, streaming blob sources.
use crate::{Error, Geometry, Result, Run, identity, invalid};
use fieldcraft_model::Model;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io::{Read, Write},
    sync::Arc,
};
pub const CHUNKED_SCHEMA: &str = "fieldcraft-external-trajectory/2";
pub const COPY_BUFFER_BYTES: usize = 64 * 1024;
pub const MAX_RECORD_BYTES: u64 = 40 * 1024 * 1024;
pub const MAX_METADATA_BYTES: usize = 16 * 1024 * 1024;
pub const MAX_BLOBS: usize = 1 + 8192 * 2 + 4096;
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BlobDescriptor {
    pub index: u32,
    pub role: String,
    pub bytes: u64,
    pub sha256: String,
}
pub trait BlobSource: Send + Sync {
    fn open(&self, index: u32) -> Result<Box<dyn Read + '_>>;
}
pub struct ArchivedChunkedTrajectoryParts {
    pub archive_key: String,
    pub binding: identity::Binding,
    pub entity_map: identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub manifest_json: Vec<u8>,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
    pub blobs: Vec<BlobDescriptor>,
    pub source: Arc<dyn BlobSource>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(crate) struct ChunkedRecord {
    pub schema: String,
    pub archive_key: String,
    pub binding: identity::Binding,
    #[serde(deserialize_with = "bounded_map")]
    pub entity_map: identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub manifest_json: Vec<u8>,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
    #[serde(deserialize_with = "bounded_blobs")]
    pub blobs: Vec<BlobDescriptor>,
    pub record_sha256: String,
}
#[derive(Clone)]
pub struct ArchivedChunkedTrajectory {
    pub(crate) record: Arc<ChunkedRecord>,
    pub(crate) source: Arc<dyn BlobSource>,
}
impl std::fmt::Debug for ArchivedChunkedTrajectory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.record.fmt(f)
    }
}
impl PartialEq for ArchivedChunkedTrajectory {
    fn eq(&self, other: &Self) -> bool {
        self.record == other.record
    }
}
struct HashWriter(Sha256);
impl Write for HashWriter {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.0.update(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl ChunkedRecord {
    fn digest(&self) -> Result<String> {
        let mut w = HashWriter(Sha256::new());
        serde_json::to_writer(
            &mut w,
            &(
                &self.schema,
                &self.archive_key,
                &self.binding,
                &self.entity_map,
                &self.result_id,
                &self.payload_sha256,
                &self.manifest_json,
                &self.admission_json,
                &self.context_json,
                &self.blobs,
            ),
        )
        .map_err(|e| Error::Invalid(e.to_string()))?;
        Ok(hex(&w.0.finalize()))
    }
    pub(crate) fn validate(&self) -> Result<()> {
        let mut size = crate::external_stream::DigestWriter::default();
        crate::codec::encode_to(self, &mut size).map_err(|e| Error::Invalid(e.to_string()))?;
        if size.part().bytes > MAX_RECORD_BYTES {
            return invalid("ExternalArchiveLimit: metadata working set");
        }
        if self.schema != CHUNKED_SCHEMA {
            return invalid("ExternalArchiveSchemaUnsupported");
        }
        if self.archive_key.trim().is_empty() || self.archive_key.len() > 1024 || self.blobs.len() > MAX_BLOBS {
            return invalid("ExternalArchiveLimit: metadata");
        }
        for (bytes, cap) in [(&self.manifest_json, MAX_METADATA_BYTES), (&self.admission_json, 1024 * 1024), (&self.context_json, MAX_METADATA_BYTES)]
        {
            if bytes.len() > cap {
                return invalid("ExternalArchiveLimit: JSON bytes");
            }
            // Validate syntax without building a JSON Value tree.
            let mut d = serde_json::Deserializer::from_slice(bytes);
            serde::de::IgnoredAny::deserialize(&mut d).map_err(|e| Error::Invalid(format!("ExternalArchiveJson: {e}")))?;
            d.end().map_err(|e| Error::Invalid(format!("ExternalArchiveJson: {e}")))?;
        }
        if self.binding.snapshot.scheme != identity::SCHEME {
            return invalid("UnsupportedIdentityScheme");
        }
        if self.binding.backend_version != identity::BACKEND_VERSION {
            return invalid("UnsupportedBackendVersion");
        }
        if [
            &self.binding.snapshot.model_revision,
            &self.binding.snapshot.reference_mesh_digest,
            &self.binding.entity_map_digest,
            &self.payload_sha256,
        ]
        .iter()
        .any(|s| !identity::digest_valid(s))
        {
            return invalid("ExternalArchiveDigestMismatch: shape");
        }
        if self.binding.snapshot.model_id != identity::model_id(&self.binding.snapshot.reference_mesh_digest) {
            return invalid("ModelIdentityMismatch");
        }
        if identity::REQUIRED_ARTIFACT_ROLES.iter().any(|k| !self.binding.artifacts.contains_key(*k))
            || self.binding.artifacts.values().any(|s| !identity::digest_valid(s))
        {
            return invalid("MissingOrInvalidArtifactDigest");
        }
        if crate::sha256_hex(&self.manifest_json) != self.payload_sha256 || self.digest()? != self.record_sha256 {
            return invalid("ExternalArchiveDigestMismatch: record or manifest");
        }
        if self.result_id != format!("fieldcraft-result/{}", self.payload_sha256) {
            return invalid("ExternalArchiveResultIdMismatch");
        }
        let mut total = 0u64;
        for (i, b) in self.blobs.iter().enumerate() {
            if b.index as usize != i {
                return invalid("ExternalArchiveDuplicateBlob: nonconsecutive index");
            }
            if !identity::digest_valid(&b.sha256) || b.role.is_empty() || b.role.len() > 1024 {
                return invalid("ExternalArchiveDigestMismatch: blob descriptor");
            }
            if b.bytes > 256 * 1024 * 1024 {
                return invalid("ExternalArchiveLimit: blob");
            }
            total = total.checked_add(b.bytes).ok_or_else(|| Error::Invalid("ExternalArchiveLimit: overflow".into()))?;
        }
        if total > crate::MAX_FILE_BYTES {
            return invalid("ExternalArchiveLimit: total");
        }
        Ok(())
    }
}
impl ArchivedChunkedTrajectory {
    pub fn new(p: ArchivedChunkedTrajectoryParts) -> Result<Self> {
        let mut r = ChunkedRecord {
            schema: CHUNKED_SCHEMA.into(),
            archive_key: p.archive_key,
            binding: p.binding,
            entity_map: p.entity_map,
            result_id: p.result_id,
            payload_sha256: p.payload_sha256,
            manifest_json: p.manifest_json,
            admission_json: p.admission_json,
            context_json: p.context_json,
            blobs: p.blobs,
            record_sha256: String::new(),
        };
        r.record_sha256 = r.digest()?;
        r.validate()?;
        Ok(Self { record: Arc::new(r), source: p.source })
    }
    pub fn archive_key(&self) -> &str {
        &self.record.archive_key
    }
    pub fn binding(&self) -> &identity::Binding {
        &self.record.binding
    }
    pub fn entity_map(&self) -> &identity::EntityMap {
        &self.record.entity_map
    }
    pub fn result_id(&self) -> &str {
        &self.record.result_id
    }
    pub fn payload_sha256(&self) -> &str {
        &self.record.payload_sha256
    }
    pub fn manifest_json(&self) -> &[u8] {
        &self.record.manifest_json
    }
    pub fn admission_json(&self) -> &[u8] {
        &self.record.admission_json
    }
    pub fn context_json(&self) -> &[u8] {
        &self.record.context_json
    }
    pub fn blobs(&self) -> &[BlobDescriptor] {
        &self.record.blobs
    }
    pub fn validate(&self) -> Result<()> {
        self.record.validate()
    }
    pub fn definition(&self) -> Result<[u8; 32]> {
        self.validate()?;
        let mut out = [0; 32];
        for (dst, pair) in out.iter_mut().zip(self.binding().snapshot.model_revision.as_bytes().as_chunks::<2>().0) {
            let s = std::str::from_utf8(pair).map_err(|e| Error::Invalid(e.to_string()))?;
            *dst = u8::from_str_radix(s, 16).map_err(|e| Error::Invalid(e.to_string()))?;
        }
        Ok(out)
    }
    pub fn validate_binding(&self, m: &Model, g: &Geometry, r: &Run) -> Result<()> {
        self.validate()?;
        self.binding().validate(m, g, r, self.entity_map(), self.binding().artifacts.clone(), &self.binding().backend_version)
    }
    pub fn open_blob(&self, index: u32) -> Result<Box<dyn Read + '_>> {
        let d = self.blobs().get(index as usize).filter(|d| d.index == index).ok_or_else(|| Error::Invalid("ExternalArchiveMissingBlob".into()))?;
        Ok(Box::new(CheckedReader { inner: self.source.open(index)?, left: d.bytes, hash: Sha256::new(), expected: d.sha256.clone(), done: false }))
    }
}
struct CheckedReader<'a> {
    inner: Box<dyn Read + 'a>,
    left: u64,
    hash: Sha256,
    expected: String,
    done: bool,
}
impl Read for CheckedReader<'_> {
    fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
        if b.is_empty() || self.done {
            return Ok(0);
        }
        if self.left == 0 {
            let mut tail = [0];
            if self.inner.read(&mut tail)? != 0 || hex(&self.hash.clone().finalize()) != self.expected {
                return Err(std::io::Error::other("ExternalArchiveDigestMismatch: blob"));
            }
            self.done = true;
            return Ok(0);
        }
        let n = b.len().min(self.left.min(usize::MAX as u64) as usize);
        let n = self.inner.read(&mut b[..n])?;
        if n == 0 {
            return Err(std::io::Error::other("ExternalArchiveMissingBlob: truncated"));
        }
        self.left -= n as u64;
        self.hash.update(&b[..n]);
        Ok(n)
    }
}
pub(crate) fn copy_blob(a: &ArchivedChunkedTrajectory, index: u32, w: &mut impl Write) -> Result<()> {
    let mut r = a.open_blob(index)?;
    let mut buf = [0; COPY_BUFFER_BYTES];
    loop {
        let n = r.read(&mut buf).map_err(|e| Error::Invalid(e.to_string()))?;
        if n == 0 {
            break;
        }
        w.write_all(&buf[..n]).map_err(|e| Error::Io(e.to_string()))?;
    }
    Ok(())
}
pub(crate) struct MissingSource;
impl BlobSource for MissingSource {
    fn open(&self, _: u32) -> Result<Box<dyn Read + '_>> {
        invalid("ExternalArchiveMissingBlob: use read_path_streamed")
    }
}

pub(crate) fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn bounded_vec<'de, D: serde::Deserializer<'de>, T: Deserialize<'de>, const N: usize>(d: D) -> std::result::Result<Vec<T>, D::Error> {
    struct V<T, const N: usize>(std::marker::PhantomData<T>);
    impl<'de, T: Deserialize<'de>, const N: usize> serde::de::Visitor<'de> for V<T, N> {
        type Value = Vec<T>;
        fn expecting(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "at most {N} entries")
        }
        fn visit_seq<A: serde::de::SeqAccess<'de>>(self, mut seq: A) -> std::result::Result<Vec<T>, A::Error> {
            if seq.size_hint().is_some_and(|n| n > N) {
                return Err(serde::de::Error::custom("ExternalArchiveLimit: sequence"));
            }
            let mut out = Vec::new();
            while let Some(v) = seq.next_element()? {
                if out.len() == N {
                    return Err(serde::de::Error::custom("ExternalArchiveLimit: sequence"));
                }
                out.push(v)
            }
            Ok(out)
        }
    }
    d.deserialize_seq(V::<T, N>(std::marker::PhantomData))
}
fn bounded_map<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<identity::EntityMap, D::Error> {
    #[derive(Deserialize)]
    struct Element {
        target: u32,
        kind: fieldcraft_model::ElementKind,
        #[serde(deserialize_with = "bounded_vec::<_,usize,64>")]
        source_nodes: Vec<usize>,
    }
    #[derive(Deserialize)]
    struct Map {
        #[serde(deserialize_with = "bounded_vec::<_,u32,100000>")]
        nodes: Vec<u32>,
        #[serde(deserialize_with = "bounded_vec::<_,Element,100000>")]
        elements: Vec<Element>,
    }
    let m = Map::deserialize(d)?;
    Ok(identity::EntityMap {
        nodes: m.nodes,
        elements: m.elements.into_iter().map(|e| identity::MappedElement { target: e.target, kind: e.kind, source_nodes: e.source_nodes }).collect(),
    })
}
fn bounded_blobs<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Vec<BlobDescriptor>, D::Error> {
    bounded_vec::<D, BlobDescriptor, MAX_BLOBS>(d)
}
