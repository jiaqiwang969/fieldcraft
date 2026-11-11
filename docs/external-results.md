# External trajectories

## Dense external static-base packages (kind 13)

`results.external.static_base.receive` receives the explicitly named
`fieldcraft-external-static-base/1` payload through an independently frozen
`fieldcraft-external-static-base/receive-profile/1`.  The profile contains the
mesh/model binding, actual SHA-256 identities for the producer, adapter and
receiving executable, reference shapes and the requested leading mode count.
Freeze it before reading a producer package; packaging then uses that profile
without changing its binding.  The package carries a static state and hold
window, measured contact state, dense K and M, the full-to-reduced constraint
map, source/admission/context JSON and optional producer modes.  Producer modes
are discarded.  The product recomputes the modal eigenproblem from K, M and T,
then checks residuals, mass orthogonality, paired MAC and the minimum paired
MAC margin.

The bounded contract is for dense packages with at most 4096 degrees of
freedom and a positive-definite reduced mass.  A hold has at least ten
consecutive no-reset frames and uses the recorded residual, velocity, drift,
inertia/internal difference and support thresholds in the kind-13 schema.
Reference shapes are required and must cover the requested modes.  Contact is
accepted only for the frozen closed bonded or closed sticking policies; every
contact refusal retains the pair/item, measured value and threshold.  Negative
tangential increments and slip ratios are format errors, and mass is checked
per translational axis rather than by summing the three axes together.

The receiver always writes `runtime_qualification: unverified`, even if a
producer claims verification.  Qualification is an independent lane-20
decision.  Frequency response remains `not_run` until a complete damping source
is supplied.  The external K/M reconstruction is not a native contact tangent;
open contact, switching and sustained sliding are rejected.  These checks are
contract and integrity checks for the bounded small example and do not by
themselves establish physical validation or a third-party reference.

`results.external.receive` receives a trajectory for the currently open model. The command
appears under **Results → Files → Receive External Results**, with fields for the package
manifest, independently frozen receive profile, unique result name and publication directory.
CLI, MCP and desktop use the same command registry. The publication directory needs both
read and write access; every source file also obeys the host's read roots.

```json
[
  {"command":"file.open","params":{"path":"incoming/input.fieldcraft"}},
  {"command":"results.external.receive","params":{
    "path":"incoming/incoming.json", "profile":"incoming/expected.json",
    "name":"impact", "store":"accepted-results"
  }},
  {"command":"results.external.list","params":{}},
  {"command":"file.save_as","params":{"path":"impact.fieldcraft"}}
]
```

Run this array with `fieldcraft-cli run receive.json`. In a separate invocation, open
`impact.fieldcraft` and call `results.external.list`. The file includes the type-13 archive,
original envelope and admission bytes, original input and mapping, frozen context, source
inventory and original receiver result identifier. Each archive keeps its own run and epoch;
no native analysis step is created. A changed binding or unsupported context prevents opening
the candidate file and leaves the current session unchanged.

The command publishes atomically, then installs the result in the session. It returns
`session_installed: true`, `project_saved: false` and measured parsing/publication times;
`file.save` or `file.save_as` persists the session. If publication succeeded but archive
construction failed, `ExternalPublishedNotInstalled` returns the actual published identifier.
The session remains unchanged and the published file remains available for diagnosis.
`results.external.remove` removes the session entry and retains those published files.
Omitting `name` is a required-field error. A provided name with no matching session entry
returns the state refusal `no external result`; the session and published files stay unchanged.

## Input contracts

The expected file has schema `fieldcraft-external-receive-request/1` and contains `run`,
`profile` and `artifacts`. Freeze it before receipt. `profile` contains `goal`,
`evidence_kind`, `source_scene`, `required_fields`, `contact_controls`, `control_evidence`,
`require_effective`, `policy_request`, `require_policy_applied`; nullable fields must be
present as explicit `null`. Supplying weaker expectations copied from an untrusted package
does not establish the requested policy or qualification.

The package manifest has schema `fieldcraft-external-package/1`, `envelope` and `sources`.
Each file specification contains `locator`, `role`, `sha256`, `bytes`, `availability`.
Locators are relative paths without parent components; symbolic links may not escape the
package. The cap is 256 MiB per input and for retained source bytes. `embedded` retains
original bytes; `identity_only` checks the bytes on receipt and retains their identity.
Executable artifacts can use `identity_only`. Frame observations, terminal evidence,
original input, entity mapping and control observations must be retained. Each run's
producer, adapter and original receiver artifacts use the identity module's public rules.
The executing receiver identity is recorded separately; missing source-commit provenance
is explicitly unknown.

The context schema is `fieldcraft-external-receive-context/1`. Reopening checks its exact
supported version, original run/profile, source hashes, current model binding, envelope
digest and independently recomputed admission. It neither allocates a new identifier nor
observes a backend on the reopening machine. Integrity checks establish consistency with
retained evidence; they are not authentication against an author who replaces all bytes
and hashes together.

## Quantities and states

Six quantity descriptors are required: position, displacement, velocity, object force,
object moment and contact energy. Values use explicit units, location, coordinate frame,
entity mapping and per-entity availability. Missing quantities use `Unavailable { reason }`;
unavailable entities have a false mask. Object moments also require their origin. Stress
is refused until its measure is defined. The command lists availability and identities;
native contour, fatigue and other native step queries do not interpret external archives.

Admission keeps transport verification, trajectory coverage, solve outcome and commit
status separate. The stored admission records `commit: not_attempted` as originally
evaluated; the context's publication receipt records `state: committed` after successful
atomic publication. File saving is a further action. Version-1 publication requires complete
solver-evidence coverage, a successful terminal receipt and all required fields. A first
failure, Newton/line-search limit, missing convergence observation, stale epoch, cancellation,
nonzero exit or signal retains the original evidence and valid prefix but prevents publication.

The context currently supports only `runtime_qualification.state: unverified`. Without a
policy request, complete recorded frame convergence can support publication without claiming
backend qualification. A policy-dependent run still needs a qualification provider; an applied
policy alone does not satisfy that gate. Unknown effective stiffness/clamping is retained and
is rejected when `require_effective` requests those observations. IPC disabled is distinct
from missing observations. Finite tangential stiffness does not establish a hard stick constraint.

## Refusal diagnostics

The following prefixes describe the failing check; they do not upgrade any evidence state.
Context and admission checks are owned by the engine, while type-13 record integrity and
binding checks remain owned by IO. Version-1 context and archive encodings are unchanged; the explicitly versioned chunked path below adds version 2.

| Diagnostic | Meaning |
| --- | --- |
| `ExternalArchivePayloadMismatch` | The envelope bytes do not match their stored payload digest when revalidating a published package. Type-13 IO may reject damaged bytes earlier with its own digest error. |
| `ExternalArchivePublicationMismatch` | The payload or result identifier differs from the frozen publication receipt, including a substituted envelope whose outer digests were recomputed. This is not a binary field checksum failure. |
| `ExternalArchiveContext` | The frozen receive context has an unsupported version or invalid shape, including a fabricated verified runtime qualification. |
| `ExternalArchiveSource` | Required retained source bytes or their source identities are inconsistent. |
| `ExternalArchiveAdmission: changed on replay` | Recomputed admission differs from the retained original admission. |
| `ExternalArchiveNameConflict` | The requested archive key conflicts with an existing result or native step. |
| `NativeContactPolicyNotApplied` | `begin` required a natively applied policy, but the provider could not verify that application. |
| `RuntimeQualificationUnverified` | Admission blocker: policy-dependent results lack independent runtime qualification, even if declaration and native application are verified. Publication remains refused. |

Stale results are reported by `stale: true` and `binding_error` in summaries; opening or
saving mismatched results reports the underlying identity error. Binary field corruption
continues to report `binary field checksum mismatch`. Missing application and unknown
qualification are separate checks; changing a diagnostic does not relax either one.

## Native lifecycle API

`external::native::NativeBackend` starts the explicitly configured interpreter as an owned
OS child. `Launch` fixes program and adapter digests, a fresh job directory, run/epoch,
scene identity and budget. The reference host is `cargo run --profile ci -p fieldcraft-engine
--example native_backend -- plan.json`; a host must first obtain its compute slot and verify
the configured environment. This reference host is separate from the receiving command.
`receive_package` is a CPU diagnostic caller for the same preparation/publication core;
it does not save a type-13 project.

The scene is uploaded once. `start_steps` / `finish_steps` check original binary output,
frame/time identity and each completion/convergence/limit observation. `reset` advances epoch,
resets the ledger and resets the native world in the same child. Cancellation becomes terminal
only after a real child wait; `shutdown` and `terminate` retain actual exit code, signal and
observer. Scope cleanup reaps the child; explicit cleanup receipts are required for a resource
release claim. Backend runtime and simulated time are separate values. Protocol fixtures test
rejection mechanics and must never be reported as a physical solve.

## Chunked native packages (version 2)

Use the same command and form. The independently frozen profile file is
`{"schema":"fieldcraft-external-receive-request/2","expected":<version-1 request>,"acceptance":"complete_only"}`.
An explicitly authorized diagnostic retention request can instead select
`"preserve_qualified_prefix"`. It admits a sealed, nonempty failed/cancelled solver prefix,
while preserving `solve: failed`/`cancelled`, the first rejected frame and the original goal.
It refuses unknown diagnostics, empty prefixes, missing originals, invalid identities and
policy qualification failures. Version 1 retains its complete-success and aggregate limits.

The manifest has schema `fieldcraft-external-package/2`, `header`, ordered `chunks`, sorted
`sources` and one nullable `terminal`. Header and manifest are each capped at 16 MiB. A header
uses `fieldcraft-external-header/2` and the version-1 envelope metadata without `frames` or
`terminal`. Each chunk has `index`, `first_frame`, `last_frame`, `metadata` and `binary` file
specifications (`locator`, `role`, `bytes`, `sha256`). Chunk metadata has schema
`fieldcraft-external-chunk/2`, `index` and `frames` with the original facts/specifications;
Present data carries `offset`, `bytes`, `sha256`, `mask` and `source`. Compact little-endian
f64 ranges are consecutive with no holes or trailing bytes. Every file and field is checked.

A chunk has at most 256 frames, 1 MiB of metadata and 64 MiB of binary data; the admission
work budget also caps each chunk's data plus masks at 64 MiB. A logical field plus mask is
at most 64 MiB and cannot be split to bypass that limit. The run has one ledger across all
chunks (up to one million frames and 8192 chunks). There are at most 4096 original sources,
each at most 256 MiB, and an 8 GiB package cap. Large raw originals can be transported as
byte-preserving source shards with an embedded reassembly manifest; field source references
name the actual retained shard. All original bytes and the whole-original digest remain
recoverable. An individual field is never reinterpreted as several smaller fields.

Publication stages a single disk spool through a 64 KiB copy buffer, validates identity,
source provenance and frame admission, and publishes under the model/run lock. The Session
owns type-13 `ArchivedChunkedTrajectory` handles. Native `file.open` and large-result saves
stream the project; no whole trajectory or ZIP byte vector is built. Saving over the same
path preserves the reader's open source handle and the old file backup. A host without a
native streaming filesystem refuses this path explicitly.

A partial project's qualified data end at `admission.coverage.last`. A failed frame and its
raw fields may remain as diagnostic originals: `observed_frames` is not the qualified frame
count. Stored admission remains `commit: not_attempted`; a successful atomic publication is
recorded separately as `committed`. Saving, reopening and byte-identical resaving do not turn
failure into numerical success or upgrade unknown runtime qualification or per-frame controls.

The receiving executable's identity is streamed independently through a 64 KiB SHA256
buffer without a file-size cap. This exception applies only to the running executable;
foreign-file, chunk and field limits remain in force. A missing embedded source commit stays
unknown even when the measured executable digest is available.
