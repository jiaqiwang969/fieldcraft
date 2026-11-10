# External backend contract foundation

`fieldcraft_engine::external` is a Rust library boundary for a separately hosted backend.
It has no CUDA dependency and does not launch a numerical solver on its own. This first
implementation is not a CLI/MCP command, a saved result type or a complete worker protocol.

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
transaction cannot release a newer lease. A future atomic publisher must execute **inside**
the commit guard. No storage publisher is currently installed, so even a matching identity
returns `StorageUnavailable`, never a successful result identifier. Mutable aliases outside
this host are not permitted by the integration contract.

`SnapshotProvider` is the adapter seam to IO `Binding::freeze` and `Binding::validate`, receiving
current Model/Geometry/Run settings, full entity mapping and actual artifact hashes. The wire
shape and transaction gate compile independently of that implementation; the current branch
still needs the authoritative IO dependency integrated by its owner. No canonicalization
algorithm is duplicated here. Fixture providers test gates/locks only. The
`external_identity_receipt` example can read a provider-produced binding and check wire
compatibility, and explicitly reports `model_identity_recomputed=false`.

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
This is source/availability consumption only; no GPU run or production identity/storage
provider is implied.

An optional `PolicyRequestPayload` freezes a `fieldcraft-contact-policy-request/1` declaration
separately from controls. It holds the wire bytes/SHA (at most 1 MiB) **and** the independently
frozen `PolicyRequest::digest`; pretty JSON's wire SHA is not that canonical digest. The
`validate_policy_declaration` provider must use IO `PolicyRequest::decode` against the current
model/geometry/run at begin and commit. The default provider refuses. No policy rules or
canonicalization are reimplemented here. Only the provider can mark a declaration verified;
the receiver keeps application and runtime qualification `Unverified` and never reports a
policy-bearing trajectory as a successful solve. `require_policy_applied=true` always returns
`NativeContactPolicyNotApplied` in this foundation, including when the declaration is valid.
This deliberately conservative quantitative gate cannot be weakened by producer fields.
Diagnostic declarations may be retained; an empty declaration is not explicit stick. A later
native adapter needs both IO `require_applied` and separate original runtime qualification
evidence after the native model/form/solver implementation is integrated. No automatic legacy
fallback, contact-state inference from warnings or commercial keyword mapping is provided.

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
| `commit = NotAttempted` | No project result has been registered or saved. `commit()` returns `StorageUnavailable`. There is no result identifier. |

Sealed incomplete data remains partial/unknown/failed/cancelled according to its evidence;
unsealed data remains staging. Future full admission requires a real solver source, complete
coverage, successful terminal evidence, purpose policy, current identity and atomic storage.
Future diagnostic prefix admission requires an explicitly allowed, sealed nonempty prefix;
it must retain the full requested domain and the first rejection, and cannot be promoted to a
full solve. Durable commits, archive reopen checks, general transport retransmission and
source-artifact authentication are not implemented in this preflight layer.

`crates/engine/tests/external_contract.rs` uses original protocol fixtures and real bounded OS
child processes. Its fabricated arrays and convergence facts are **not numerical validation**.
The subprocess tests exercise exit codes, signals and reaping without loading a GPU backend.
