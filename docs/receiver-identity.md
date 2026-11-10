# Receiver snapshot identity

`fieldcraft_io::identity` supplies exact, content-addressed receiver snapshots and full-mesh
entity-map validation. This is a library contract for receiver admission and commit, not a
new result type or an external solver implementation.

Scheme `fieldcraft-receiver-snapshot/1` uses SHA-256. Each hash starts with its domain's UTF-8
bytes, followed by each part's u64 little-endian byte length and bytes. The reference hash
(domain `fieldcraft-reference/1`) covers the production `MCMESH02` encoding and compact JSON
of the model's ordered set map. Labels, SI reference coordinates, element types, ordered
connectivity and geometry classification are included. No tolerance equivalence is applied.

`model_id` is `sha256:` followed by this reference hash. The revision hash (domain equal to
the scheme) covers the reference hash's ASCII hex, compact JSON of `model.without_mesh()`,
the geometry source-content tuple `[design_sha_or_null, mesh_sha_array]`, and compact JSON of
`Run` (backend preferences and fallback). Transient command overrides must be frozen
separately by the runtime contract. This deliberately includes conservative differences such as display units. Paths,
views, journal, source replay text and result values are excluded. Serialization changes
that change this contract require a new scheme.

These are snapshot identities, not document lineage: identical reference families share a
model ID; edits to definitions change the revision; editing the reference changes the model
ID; undoing to exactly the same content restores its identity. Copies and save-as preserve
content identity. Applications needing distinct document lineage must use a separate contract.

Production archive writing records the snapshot in the format-3 manifest's optional
`identity` field. Reading recomputes and validates it after loading the mesh. Legacy archives
without this field remain readable, but do not acquire a claimed historic binding. Readers
that reject unknown manifest fields may reject these new archives. Result encodings are
unchanged.

`EntityMap` uses dense source indices and explicit target labels. All nodes and elements must
be covered exactly once. Element kinds and ordered connectivity after mapping must agree.
The digest covers compact JSON with domain `fieldcraft-entity-map/1`. A source permutation
changes the map identity even if its connectivity is adjusted consistently. Partial maps,
interpolation and implicit object ordering are unsupported. Adapters must separately bind
any multi-object flattening table and source scene.

`Binding::freeze` records the snapshot, map digest, actual producer/adapter/receiver artifact
SHA-256 values and backend version (currently `0.0.29`). `artifact_digest` hashes supplied
bytes; the host must read the actual artifacts, not trust self-reported hashes. Call
`Binding::validate` at both admission and commit with current model, settings, map and
observed artifact identities. Hold the receiver's model lock across commit validation and
publication. This library does not implement that lock, epochs, frame status, publication or
source-scene verification; those remain runtime-manager obligations.

Tests in `crates/io/tests/identity.rs` exercise production archive round trips, forged archive
identity, changed reference/definition, duplicate nodes, connectivity/index errors, source
permutation and program/version mismatch. Synthetic artifact bytes test identity checking,
not an installed backend or its physical capabilities. Effective contact stiffness and its
clipping status require source observations; requested values cannot substitute for them.

## Shared shape rules

Consumers import `fieldcraft_io::identity::{SCHEME, BACKEND_VERSION, MODEL_ID_PREFIX,
REQUIRED_ARTIFACT_ROLES, model_id}`. The backend version remains `0.0.29`, model IDs
remain `sha256:` followed by the reference mesh digest, and required artifact roles
remain `producer`, `adapter`, `receiver`. Additional roles are allowed but all digests
are checked. `model_id` only formats an existing digest; it does not establish its
validity or ownership. Use `Binding::freeze/validate` to recompute authoritative identity.
These exports do not change serialized snapshots, revisions or runtime evidence.

`fieldcraft_io::identity::digest_valid(&str) -> bool` is the shared digest-shape check:
exactly 64 lowercase ASCII hexadecimal digits, without trimming, prefixes or case normalization.
Binding artifacts and capability observation sources use this same check. All-zero text is
shape-valid; this function does not verify artifact content, source provenance or runtime qualification.
