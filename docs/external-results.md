# External trajectories

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
atomic publication. File saving is a further action. Publication requires complete
solver-evidence coverage, a successful terminal receipt and all required fields. A first
failure, Newton/line-search limit, missing convergence observation, stale epoch, cancellation,
nonzero exit or signal retains the original evidence and valid prefix but prevents publication.

The context currently supports only `runtime_qualification.state: unverified`. Without a
policy request, complete recorded frame convergence can support publication without claiming
backend qualification. A policy-dependent run still needs a qualification provider; an applied
policy alone does not satisfy that gate. Unknown effective stiffness/clamping is retained and
is rejected when `require_effective` requests those observations. IPC disabled is distinct
from missing observations. Finite tangential stiffness does not establish a hard stick constraint.

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
