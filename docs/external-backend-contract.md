# External backend contract foundation

`fieldcraft_engine::external` is a Rust library boundary for a separately hosted backend.
It has no CUDA dependency and does not launch a numerical solver on its own. The production
seams below are library APIs, not a CLI/MCP command or a complete worker protocol.

`run::RunLedger` is an in-memory state machine. A `RunKey` binds a temporary run, epoch and
worker generation; it is not a persistent model identity. The ledger distinguishes native
completion, convergence, Newton and line-search limits, world validity and snapshot completion.
Every diagnostic is `Observed::Known { value, source }` or `Unknown { reason }`; absent JSON
fields fail decoding. Frame zero belongs to initialization. Advanced frames start at one and
must be contiguous, with strictly increasing finite seconds and the frozen final time.

`begin_advance` reserves a bounded batch, `accept_frame` checks each receipt, and `finish`
requires an explicit terminal source after the entire goal. Completing a batch or exiting zero
does not finish a solve. The first invalid frame locks the ledger and preserves its facts;
subsequent frames or advances cannot repair the prefix. Missing/unknown observations remain
distinct from a known numerical failure. `request_cancel` only enters `CancelPending`.

On native hosts, `worker::OwnedWorker` launches an explicitly authorized direct child, observes
its exit through `Child::try_wait`, and reaps it. Receipts record the program and arguments,
run key, child and parent PIDs, observation time, direct-child layer, exit code or signal, and
the raw POSIX wait status where available. An ordinary exit 139 is not inferred to be SIGSEGV.
`ProcessExit` cannot be deserialized from wire claims. Cancellation only becomes terminal when
an exit is observed. Reset requires that actual receipt, a new worker generation and an
incremented epoch; it returns to initialization and clears the old in-memory prefix. Callers
must archive the old receipts first. Reset is not a determinism guarantee.

This host seam owns **one direct child only**. It is not a process-tree manager: do not pass a
launcher that leaves descendants alive. Graceful native cancellation, process-group policy,
bounded log capture, persistent deduplication, checkpoint recovery and the real backend wire
adapter remain integration work. Pipe streams are disabled to avoid blocking on an unread
pipe. Explicit `terminate_and_reap` returns evidence; `Drop` only provides best-effort cleanup.
The receipt makes no statement about GPU memory or an outer scheduler's lock release.

`receive::ExternalReceiver<I>::validate(expected_model, active_run, envelope)` is a bounded
preflight entry. `IdentityVerifier::verify` is the seam for the model owner's persistent
revision, reference mesh, executable/build identity and entity-map policy. Values in
`ModelBinding` are opaque; there is no locally invented hash canonicalization. The default
`UnconfiguredIdentity` refuses with `IdentityUnavailable`. String equality is insufficient
for a production implementation of that trait. Revalidate identity at any future commit.

The binding wire shape follows `fieldcraft-receiver-snapshot/1`: `snapshot` contains scheme,
model_id, model_revision and reference_mesh_digest; the binding also contains entity_map_digest,
producer/adapter/receiver artifact hashes and backend_version (`0.0.29`). `validate_shape`
checks this shape and supported version, **not** the model content. Canonical digest generation
and full-mesh connectivity validation belong to the IO identity implementation. The node array
maps dense source indices to target labels; element rows include the target, ElementKind and
ordered source-node indices. The separate object map stays in the runtime envelope.

`transaction::ReceiverHost<P>` owns the current model context, active run and a mutex.
`begin(run, entities, profile)` takes an exclusive receiver lease and calls `SnapshotProvider`
freeze/validate under that lock. The profile freezes the requested frame/time goal, source-scene
reference, evidence kind and required fields. `ReceiveTransaction::validate` and `commit` check
them again; a producer cannot shorten the goal to make an incomplete trajectory look complete.
Model edits use the same lock, and are permitted between checks: changed identity is refused at
commit. Reset/replacement invalidates an older run/epoch/generation and lease; dropping an old
transaction cannot release a newer lease. `ReceiverHost::with_store` holds a receiver-owned
`ResultStore` and invokes it while the commit lock is held; `ReceiverHost::new` remains the
deliberately unconfigured path and returns `StorageUnavailable` after successful validation.
Mutable aliases outside this host are not permitted by the integration contract.

`SnapshotProvider` is the adapter seam to IO `Binding::freeze` and `Binding::validate`, receiving
current Model/Geometry/Run settings, full entity mapping and actual artifact hashes. The wire
shape and transaction gate compile independently of that implementation. The production
`external::production::IoSnapshotProvider` delegates to the merged `fieldcraft_io` identity
codec and does not duplicate canonicalization; it also delegates controls and policy decoding.
Fixture providers remain test-only. The `external_identity_receipt` example can read a
provider-produced binding and check wire compatibility, and explicitly reports
`model_identity_recomputed=false` because it is only a wire reader.

`production::FileResultStore` is the minimal receiver-owned durable seam. It accepts only a
`Solver` admission with complete coverage and `SolveOutcome::Succeeded`, serializes the envelope
and admission, writes one self-checking JSON record through `fieldcraft_io::fs::write_atomic`,
and returns `fieldcraft-result/<sha256>` only after that write succeeds. Reopening verifies the
schema, identifier and envelope digest; publishing the same bytes is idempotent and a digest
collision is rejected. This is a library store, not a project result-kind allocator or a
240-frame production entry point.

The profile optionally freezes the bytes and SHA-256 of a
`fieldcraft-contact-controls/1` receipt plus a receiver-owned `require_effective` policy.
Bytes are capped at 64 KiB and their digest is checked. `SnapshotProvider::validate_controls`
must delegate decoding/validation to IO `ContactControls`, then authenticate Known sources
against the current run, epoch, scene/config and artifacts. It runs under the same lock at
begin and each later validation. Its default implementation refuses. Quantitative use requires
a receipt and known effective kappa and clamp observations; a refusal never retries with a
weaker policy. A diagnostic transaction may retain Unknown when its provider permits it.
The envelope carries the frozen receipt digest; substituting another receipt is rejected.
The IO observation wire type (`status`, `source_sha256`, `locator`) differs from the runtime
`Observed` type (`state`, nested `SourceRef`): translate explicitly, never alias these codecs.
This boundary does not infer effective kappa from requested kappa, penalty scale or silence.
The parameter-capability report (`fieldcraft-parameter-capabilities/1`) and native archive
validation remain IO responsibilities; no additional archive fields are written here.

Route-20 evidence is consumed through `ControlEvidenceReceipt`, alongside (never in place of)
the raw controls bytes. It binds the producer run label, epoch, observed compute PIDs, the
complete `bindings.json` SHA/locator, and the controls-file SHA/locator. `scope=initialization`
means exactly the recorded initialization observation point; it does not supply any value for
an advanced frame, even when both `kappa_effective_pa` and `kappa_clamped` are Known. The
receipt's `observed_frame` and `observed_time_s` are optional as a pair, so their absence is
preserved as unknown rather than filled from a requested timestep. `scope=advance_frames` is
the only scope that can satisfy `require_effective=true`, and it must carry both Known values
with solver source references. `native_ipc=disabled` is a separate availability state: it has
no controls file, no PID/epoch, and both native-kappa observations remain Unknown; it is not
the same as an ordinary Unknown initialization receipt.

The receiver rejects a wrong run label/epoch, missing or changed record digest, mismatched raw
controls digest, copied source, or a quantitative request backed only by initialization,
Unknown, or IPC-disabled evidence. Diagnostic admission retains the exact initialization
value, Unknown reasons, or IPC-disabled state in `Admission.control`; it does not turn those
states into a solve or a result identifier. The shared 0.0.29 five-case audit currently has
10 initialization Known records, 70 initialization Unknown records and 10 IPC-disabled
records, with all 80 non-IPC trajectories still lacking per-frame effective/clamp values.
This route-20 receipt remains source/availability consumption only; it is now carried through
`ReceiverHost` without upgrading its observation state. The production identity/store seam below
is independent of the still-missing per-frame observations; no GPU run is implied.

An optional `PolicyRequestPayload` freezes a `fieldcraft-contact-policy-request/1` declaration
separately from controls. It holds the wire bytes/SHA (at most 1 MiB) **and** the independently
frozen `PolicyRequest::digest`; pretty JSON's wire SHA is not that canonical digest. The
`validate_policy_declaration` provider must use IO `PolicyRequest::decode` against the current
model/geometry/run at begin and commit. The production provider calls IO `PolicyRequest::decode`
and, when `require_policy_applied=true`, IO `require_applied`; the receiver marks
`application=Verified` only when that native field is present in the current model. Runtime
qualification remains `Unverified`, and a declaration alone cannot make a trajectory a
successful solve. Providers without this seam still refuse with
`NativeContactPolicyNotApplied`; a native application failure keeps that same named refusal.
This conservative quantitative gate cannot be weakened by producer fields. Diagnostic declarations
may be retained; an empty declaration is not explicit stick. No automatic legacy fallback,
contact-state inference from warnings or commercial keyword mapping is provided. Finite penalty
stiffness is not treated as hard stick, and runtime qualification still requires original solver
evidence.

The envelope declares run/epoch/generation, run goal, source scene, source evidence kind,
required-field profile and terminal source. Each frame explicitly declares position,
displacement, velocity, object force, object moment and contact energy, either present or
unavailable with a reason. The first profile accepts SI (`m`, `m/s`, `N`, `N*m`, `J`) in
`world_z_up`. Object moments require their origin in metres. Positions are current coordinates;
displacements refer to the bound reference mesh. Any other unit or coordinate frame is rejected,
not silently transformed. Stress is rejected until its measure/location contract exists.
Object forces are not node reactions; contact energy is not total energy.

Present binary blocks contain little-endian f64 values with SHA-256, an entity availability
mask and a source. Values are compact: only true mask rows have values. Unknown rows contain
neither zero nor NaN placeholders. Field counts, entity counts, frame counts and aggregate
binary/mask bytes are capped, and available values must be finite. A future network decoder must
apply its own framing/decompression caps **before** allocating the typed envelope.

The returned states are independent:

| State | Meaning |
|---|---|
| `transfer = Verified` | The submitted block checksums, shapes and structural checks passed. A structural error returns a named error, not an admission. |
| `coverage` | Empty, a contiguous qualified prefix, or all requested frames; never renumbered to hide a gap. Missing required data or a first frame failure cuts the prefix. |
| `solve` | Qualified source facts plus a terminal claim for the entire goal, distinct from byte integrity. A protocol fixture can never produce `Succeeded` at the receiver. |
| `commit = NotAttempted` | The admission object is the pre-storage report. With `ReceiverHost::new`, `commit()` returns `StorageUnavailable`; with `with_store`, the atomic store returns a result identifier only after a complete solver-success write. |

Sealed incomplete data remains partial/unknown/failed/cancelled according to its evidence;
unsealed data remains staging. Future full admission requires a real solver source, complete
coverage, successful terminal evidence, purpose policy, current identity and atomic storage.
Future diagnostic prefix admission requires an explicitly allowed, sealed nonempty prefix;
it must retain the full requested domain and the first rejection, and cannot be promoted to a
full solve. The minimal file-store reopen check is implemented by `production::FileResultStore`;
project result registration, archive indexing, general transport retransmission and
source-artifact authentication remain outside this layer.

`crates/engine/tests/external_contract.rs` uses original protocol fixtures and real bounded OS
child processes. Its fabricated arrays and convergence facts are **not numerical validation**.
The subprocess tests exercise exit codes, signals and reaping without loading a GPU backend.

## Retained interface fixtures

The `production_identity_lock_commits_and_reopens_a_real_file_store` test can retain a complete
package by setting `FIELDCRAFT_FIXTURE_EXPORT` to a **new** directory (its parent must exist).
The package includes the original `.fieldcraft` model/geometry/run, entity map, frozen request,
input envelope with binary arrays, binding, admission, exact stored result, source-marker bytes
and the CPU test executable. All three artifact roles identify that same executable by its
actual bytes. The model archive does not contain the external result; the separate `store/`
record is the library store's artifact. No project result kind is allocated by this example.

These are **protocol fixtures invoking real identity and persistence APIs**, never libuipc
solve evidence. The fixture injects `Solver`/`Succeeded` wire claims to exercise the existing
store gate. `package.json` separately declares `evidence_class=protocol_fixture`,
`runtime_qualification=unverified`, and `numerical_solve_verified=false`. Do not infer numerical
convergence from the injected labels, constant arrays, exit zero or successful file reopening.
A real trajectory with authenticated frame observations must traverse the receiver before any
numerical-evidence claim can be upgraded.

Run a separate reader process with:

```sh
cargo run --offline -p fieldcraft-engine --example external_fixture_reopen -- /path/to/package
```

It hashes the retained files and program, independently reads the original model, recomputes
its IO identity and the frozen mapping, checks the request and original envelope, calls
`FileResultStore::reopen`, then recomputes and compares the admission. This is readback, not a
second commit. On reopen the stored envelope digest must also equal the identifier suffix;
rehashing a changed payload under the old identifier is rejected. Hashes provide content
identity, not signatures or protection against replacing the entire trusted package index.
