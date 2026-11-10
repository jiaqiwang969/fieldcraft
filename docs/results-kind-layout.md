# Unified version 07 results

The only current writer uses **MCRSLT07**. The little-endian envelope is eight
ASCII magic bytes, `u32 step_count` (at most 10,000), then name-sorted records:
`u64 name_bytes`, UTF-8 name, 32 definition-hash bytes, `u8 kind`,
`u64 payload_bytes`, bounded serde binary codec payload. No layout sentinel,
optional trailer, alternate development-kind aliases or trial decoders are used.
The ZIP manifest verifies part sizes and SHA256.

| Kind | Version 07 payload |
|---|---|
| 0 | StaticResult without fatigue, including pretension and nested nonlinear/contact results |
| 1 | FrequencyResult |
| 2 | HeatResult |
| 3 | ExplicitResult |
| 4 | FluidResult |
| 5 | HarmonicResult including mass and operator_diagonal_bounds |
| 6 | Full Dynamics Response: frames, spectrum flag, natural frequencies, warnings, energy ledger and mass |
| 7 | Tuple (StaticResult, FatigueField) |
| 8 | Tuple (HarmonicResult including mass and operator_diagonal_bounds, FatigueField) |
| 11 | Nonlinear total-Lagrangian St. Venant–Kirchhoff rod2 History |
| 12 | FrequencyResult with explicit `mass_kind` and `section_mass_policy` provenance |

Fatigue is skipped in the base result's codec and encoded exactly once as the
second tuple member. Kinds 5 and 8 always carry mass, including consistent results
without an operator diagnostic. Kind 6 stores the complete direct/modal history
or response-spectrum result; it is not a placeholder refusal. There are no current
kinds 9 or 10. Additional kind assignments require Router approval.

Kind 1 remains the legacy frequency layout. Its decoded mass identity is explicitly
`unrecorded`; readers never infer `consistent` from the model or solver defaults. New
frequency solves write kind 12 and preserve the selected `consistent` or `lumped`
mass policy plus the section-mass policy string. Kind 12 adds no fatigue fields.

For decoders, type 5 field order is `mass, operator_diagonal_bounds, step, nodes,
reference_points, frames, natural_frequencies, residual_vectors, stats, warnings`.
Type 6 order is `mass, step, nodes, frames, spectrum, natural_frequencies, warnings,
energy`. The fatigue member of 7/8 is `damage, life_seconds, inputs_json,
dependencies`. The codec writes structure-field presence bytes, little-endian
fixed-width numbers, `u64` list/string lengths, and `u32` enum variants; skipped
fatigue fields in the base types emit no member. A complete decoder must consume
the entire length-delimited payload, including optional values and nested fields.

## Old version 05 and 06

Only kinds **0–5** are accepted using their explicit old schemas:

- 05/static uses the frozen pre-pretension static structure and restores an empty
  pretension vector. 06/static uses the pre-merge prestress structure including
  pretension (the frozen `fusion-p4a` producer).
- 1–4 retain their original payloads in both versions.
- 05/06 harmonic kind 5 has field order step, nodes, reference_points, frames,
  natural_frequencies, residual_vectors, stats, warnings. It restores consistent
  mass and no operator diagnostic; it is not read as version 07 kind 5.
- Every old kind **6 or above** is refused before decoding its payload: the error names
  the step, the kind and the older version, calls these ambiguous development layouts and
  asks for the file to be regenerated with the program that wrote it.

The pre-merge fatigue producer also wrote static kind 0 under 06 without the
prestress field. That incompatible file is not guessed or retried: retain it and
recover using its verified original program. Accepting version 06 does not mean
all incompatible development producers had the same static schema. Historical
06/7 or 06/10 dynamics and fatigue collisions never receive aliases in this reader.
The withdrawn intermediate unified-06 sentinel experiment is not an accepted
schema. Other unsupported magic versions retain the existing result-drop note.

Keep each original file, source/program identity and SHA256. Where a result is
still needed, use its verified original program to export a result-free model and
reference replies, or use an independently retained original command recipe;
regenerate through the version 07 native solver/writer and compare all fields.
Never change old magic/kind bytes as migration. Current built-in test fixtures
are generated under MCRSLT07; old05/06 fixtures stay in their historical folders.
Do not recalculate large BASE responses merely to repair metadata.

## Mass and cross-lane integration

`solve.transient` (direct/modal), `solve.spectrum` and `solve.harmonic` share
`MassKind`: `consistent|lumped`, default `consistent` on each solve. The selection
feeds mass assembly, damping and modal projection and is stored in types 5/6/8.
The random-frame pure evaluator and unaveraged element recovery coexist with
session-bound fatigue persistence. Random/fatigue inherit their solved source's
mass; a separate random `mass` override is refused. Imported modal random steps
retain their supported default-consistent contract and refuse external overrides.
No PSD is turned into a signed tensor history or nonlinear principal-stress spectrum.

`solve.run mass` retains its scoped contract: only frequency, transient and
response-spectrum steps together; other mixed cases use individual commands.
The integration does not start lane 17's separately scheduled 5R or any BASE run.

## Native evidence tool

`cargo run --release -p fieldcraft-engine --example results_roundtrip -- input.fieldcraft new-directory`
opens through the native reader, rewrites through the native writer, compares the
entire decoded result objects, and exports complete fields plus archive hashes.
It does not solve, infer a legacy producer, repair invalid data or replace a
physical validation. The built-in fixture generator and IO synthetic-layout tests
separately cover old compatibility, all current kinds and explicit refusals.

## Nonlinear rod kind 11

Kind 11 is independently allocated for the rod2 total-Lagrangian St. Venant–Kirchhoff
history. The magic stays MCRSLT07 and the archive stays fieldcraft/3. No 08, kind 9,
kind 10, archive /4 or payload-version tag is introduced. Existing kinds 0–8 retain
their exact schemas. Old 05/06 kinds 6 and above remain ambiguous and refused,
including 9/10/11. A unified-07 reader predating kind 11 rejects it by name. Pre-07 readers may instead
open the model without results under their existing unsupported-magic policy.

The `transient::nonlinear::History` payload follows declaration order:

| Field | Codec type / shape | Meaning and SI unit |
|---|---|---|
| step | string | Model step name |
| mass | MassKind enum u32 | consistent=0, lumped=1; constant reference operator |
| settings | string containing canonical Transient model JSON | Actual dt (s), increments, output stride, Newmark/HHT choice, Rayleigh alpha (1/s), initial u (m)/v (m/s), load-amplitude definitions; no guessed required fields |
| max_iterations; tolerance | usize/u64; f64 | Newton correction ceiling and relative force criterion |
| nodes | Vec<u32> | Ascending node labels |
| elements | Vec<RodIdentity> | label:u32, nodes:Vec<u32> (2 ordered), reference_coordinates:Vec<[f64;3]> (2, m), reference_area:f64 (m²), young:f64 (Pa) |
| frames | Vec<Frame> | Accepted output states, initial and final included |
| accepted_steps | Vec<AcceptedStep> | Every accepted increment including initial: time:f64 (s), newton_corrections:usize, relative_residual:f64 |
| warnings | Vec<string> | Scope and interpretation |

Frame order: `time:f64` s; `displacement,velocity,acceleration:Vec<[f64;3]>`
(m, m/s, m/s²); `internal_force,external_force:Vec<[f64;3]>` N (assembled global
Cartesian nodal fields, not support reactions); `rods:Vec<RodRecovery>` following
element order; `kinetic_energy,strain_energy,external_work,physical_dissipation,
energy_balance:f64` J; `newton_corrections:usize,relative_residual:f64`.

RodRecovery order: `green_strain:f64` dimensionless, `second_piola_stress:f64`
Pa (reference axial material coordinate), `nominal_stress:f64` Pa (axial first
Piola scalar along the current rod relative to reference area), `axial_force:f64`
N, `nodal_internal_force:Vec<f64>` (six global Cartesian N, ordered endpoints),
`strain_energy:f64` J. These fields are uniform in the two-node element; the
recovery location is its reference midpoint, not an extrapolated nodal stress.
Reference/current axes and midpoint are uniquely obtained from reference coordinates
and accepted nodal displacements. No Cauchy stress or current-area law is claimed.
S=Young*E_G, P=lambda*S and N=A0*P. The reader recomputes all rod fields from u and
checks their identity, and checks assembled internal forces and convergence logs.

T=0.5*v^T*M*v; V=sum(0.5*Young*A0*L0*E_G²); W is accumulated trapezoidal dead-force
work; D is accumulated trapezoidal v^T*C*v with C=alpha*M. Balance=T+V−(T0+V0)+D−W
includes integration/work-quadrature error, not physical damping. HHT numerical
loss is not a separate exact physical-dissipation channel. Initial acceleration
satisfies reduced M*a0=f0−f_int(u0)−C*v0. Its initial residual is diagnostic;
subsequent accepted residuals must satisfy the stored Newton tolerance. Only
successful complete histories are native results; failure diagnostics stay in the
command error/local run log, never in accepted frames.

Required new fields have no missing-field defaults. The existing envelope's
model-freshness definition hash is unchanged; no program, commit or additional
input provenance hash is added. Those identities belong in external handoff
manifests. Unknown kinds, damaged payloads, invalid dimensions, incomplete frames,
nonfinite state and inconsistent recovered stress are refused. The existing
unsupported-magic result-drop policy is unchanged.


## Kind 12

Kind 12 is a frequency result produced by `solve.frequency` in the unified 07 layout. Its payload is the FrequencyResult schema with `mass_kind` (`consistent` or `lumped`) and `section_mass_policy` explicitly recorded. Kind 1 remains the frozen legacy frequency payload; missing metadata is exposed as mass identity `unrecorded`, never inferred from the model or step. Kind 12 does not alter kinds 0–11, and it carries no fatigue fields.

## External backend trajectories (kind 13)

Version 07 stores `external_results::ArchivedTrajectory` under kind 13 without a
native solver step. The outer name is `archive_key`; the outer definition bytes
are the bound model revision, not `definition_hash` of a fabricated step.
The payload uses `fieldcraft-external-trajectory/1`. Its record checksum is SHA256
of the codec tuple `(schema, archive_key, binding, entity_map, result_id,
payload_sha256, envelope_json, admission_json, context_json)` in this exact order.
The checksum is excluded from itself. Three JSON byte arrays are preserved,
not reserialized; each is capped at 256 MiB, within the existing results cap.
The envelope checksum and original result ID must agree. Names are nonempty,
at most 1024 UTF-8 bytes, and cannot collide with native step names.

IO validates structure, integrity and the existing snapshot/entity binding on
both project write and read. Receiver-owned frame, context and admission
semantics are revalidated by the engine before installing a reopened project.
Opaque JSON accepted by IO alone is not a qualified solver result. Unavailable
fields and unknown observations remain in their original payload bytes. A
self-consistent digest is not authentication of a source; producer artifacts
must be checked against independent expectations at admission. Old producers
need not equal the current reader. No result layout other than kind 13 changes.

### External trajectory schema 2 (kind 13)

The kind remains 13 in MCRSLT07. Schema 1 and kinds 0–12 retain their wire
bytes. Schema 2 is explicit, never selected by input size. The codec payload
starts with `fieldcraft-external-trajectory/2`, followed by archive key, binding,
entity map, result ID, manifest SHA256, original manifest/admission/context JSON
bytes, ordered blob descriptors, and record SHA256. The record digest is SHA256
of the compact JSON tuple of those fields except the digest itself. Source handles
and filesystem paths are not serialized in the record digest. The result ID is
`fieldcraft-result/<sha256 of original manifest bytes>`.

`external_results::ArchivedChunkedTrajectory` owns an `Arc<dyn BlobSource>`;
its read-only accessors expose metadata, never a complete trajectory buffer.
Blob descriptors contain consecutive indices, roles, byte counts and SHA256.
Stored ZIP members are named `external-<sha256>.bin`, sorted by name, with fixed
ZIP timestamps and permissions. Identical content is deduplicated. Every member
must be declared and referenced; missing, duplicate, extra, compressed, truncated,
oversized or changed blob content is rejected. Blob streams check exact length,
EOF and digest, including when saving again after a source changes.

Use `write_with_to` with a seekable **atomic temporary destination**, flush/sync,
then rename. Do not truncate a source project in place. `read_path_streamed` owns
the opened file handle; renaming or replacing its path does not redirect its blob
reads. It validates the parts and model binding before returning. Results metadata
is streamed through the codec; blob copying and hashing use 64 KiB buffers and
one active blob stream per traversal. The legacy byte-vector writer/reader remain
for small projects; chunked records require the streaming project APIs.

Manifest/context JSON are limited to 16 MiB each, admission JSON to 1 MiB, blob
sources to 256 MiB each, and the project to 8 GiB. Ordered descriptors are bounded
to 20,481. Transport field/chunk shape, 64 MiB logical field protection, frames,
terminal state, qualification, and replay of admission are receiver responsibilities;
IO preserves unavailable and partial/failed state bytes without asserting success.
The IO evidence runner is not a formal receiver and cannot authorize publication.
Frozen older readers reject new ZIP members through their existing diagnostics;
new readers reject unknown type-13 schemas with `ExternalArchiveSchemaUnsupported`.

To keep the IO working set bounded across multiple external records, schema-2
encoded metadata totals at most 40 MiB per project (leaving room within the 64 MiB
IO budget for decoded containers and buffers). Oversized metadata is rejected
before stream decoding. Entity mappings admit at most 100,000 nodes/elements and
64 connectivity entries per element; blob descriptor counts are checked before
sequence allocation. Individual JSON limits still apply. This is a storage
resource limit, not a change to receiver admission or physical applicability.
