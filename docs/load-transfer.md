# Load-transfer result histories

The load-transfer entry receives a result-kind-13 bundle through the common
external-results registry. It verifies the current model binding, recomputes
applicability from the raw activation frames, runs the force, moment, virtual
work, impulse and angular-impulse checks, and can save the received archive in
the project. The archive is retained as the original envelope, so opening and
resaving the project preserves its bytes.

```json
[
  {"command":"file.open","params":{"path":"support.fieldcraft"}},
  {"command":"results.external.load_transfer.receive","params":{
    "path":"incoming/load-transfer.json",
    "name":"support-load",
    "project":"support-with-load.fieldcraft"
  }},
  {"command":"results.external.load_transfer.apply","params":{
    "name":"support-load",
    "step":"support-load-transient"
  }}
]
```

`receive` reports per-frame conservation, time-integral conservation,
geometry/adoption applicability, force-precision limitations, runtime
qualification and the backend crash receipt. A producer-claimed verified
qualification is rejected; qualification can only be established by an
independent receiver-side record. The reply also reports the actual SHA256
identities of the producer artifact, adapter artifact and receiving executable.
When `apply` reopens a saved project, it checks that the producer and adapter
identities still match the received record. `receiver_sha256` is the executable
that received and persisted the record; `current_receiver_sha256` is measured
when applying it. Those two receiver identities may differ, so a saved project
can be reopened on another program version or machine without weakening the
producer/adapter identity check.
`apply` creates native unit-force loads with tabular amplitudes. When the named
step already exists and is a transient step, the loads and amplitude tables are
added to that step while its boundary conditions, integration, method, damping,
output cadence and other loads stay unchanged. When the step is absent, `apply`
creates a transient step; pass `boundary_conditions` to enable existing
supports. The reply reports the effective settings and boundary-condition
count, and warns when a newly created step has no supports. The default
integration is average acceleration, the default method is direct, and the
default damping is zero.

Admission is atomic: a failed force, moment, virtual-work, impulse or
angular-impulse check names the first frame or integral item, actual residual
and tolerance before the external result is installed. `apply` repeats the
same checks. It preflights the model definition limits before cloning the
model, creates one set per active node and one force load per nonzero mapped
component, and omits components that are zero for the complete history. A
failed applicability result, nonuniform time axis, a mismatched existing step
time grid, missing mapped node, a non-transient existing step, or insufficient
history is rejected before the cloned model is published. An existing
transient step keeps its settings; if explicit settings or supports are
supplied, they must match the existing values. Integration, solution method,
damping and output cadence are accepted for a new step and are returned in
`transient_settings` for the solver receipt. The mass formulation is selected
by `solve.transient` (`mass:"consistent"` or `mass:"lumped"`), not by
`load_transfer.apply`. The incoming time
grid must be uniform; existing-step grids must have the same interval count
and time step (within the documented rounding tolerance), and no implicit
interpolation is performed. The time-axis intervals must agree within
`1e-12 s * max(1, |interval|)`; for very small time steps the absolute part is
`1e-12 s`. This is the admission tolerance for accumulated floating-point
roundoff. The step time is always computed from the complete
duration, `(last_time - first_time) / (frames - 1)`, after that check rather
than copied from the first interval. The reply exposes this source and
tolerance.

The full product regression in `crates/engine/tests/real_load_transfer.rs`
opens a structural model, retains its reference transient step untouched,
receives the point-force frames, applies the mapped native loads to a new step
with the same supports and integration settings, solves that step, probes the
result, and writes the raw product reply before comparison. Its time-axis
comparison allows `1e-12 s` for floating-point rounding; the five reference
checks use a relative threshold of `1e-9`. An independent three-dimensional
comparison retains the three required windows around 0.43%, 0.43% and 0.06%.

The regression records two distinct comparisons. A direct native-load control
with the same supports and integration, using only the incoming amplitude
samples, is numerically identical to the receive/apply product path (all probe
values have zero difference). Both differ from the reference transient because
the reference uses a denser source sampling grid while the incoming result uses
1,025 structural frames. The five-value reference gate remains `1e-9`; the
observed values are recorded as four values below `1.2e-10` and the largest
stress value `2.9864501609485475e-8`, rather than being reported as a pass. The
contract uses one incoming frame per structural step and does not claim support
for a source sampled more densely than that step.

The receive path rejects a failed frame or time-integral conservation check
before installing the archive and includes the first item, measured residual
and tolerance in the error. Apply preflights the complete set/load object
count, creates one set per active node, omits components that are zero over
the whole history, and reports a named error before mutating the model. The
entity map reports a missing node reference instead of indexing through a
missing entry. These paths and the malformed-input rejection matrix are
covered by `crates/engine/tests/load_transfer_entry.rs`.

For every frame, the incoming nodal force must equal the product's `Hᵀ`
recomputation exactly, using the same operation order; an external producer
must follow that order when constructing a bundle. Uniform time-grid checks
use an absolute `1e-12 s` tolerance when the time step is very small. Virtual
work is an identity of the product's `Hᵀ` implementation and is not evidence
of an independent validation; force and force moment are the meaningful
conservation evidence.

The frozen tolerances are relative `1e-10` with absolute `1e-12` for the
conservation checks, activation-area change at most 5%, changed-component
weight at most 5%, gap change at most 2%, and peak-sampling loss at most 1%.
Missing per-frame effective kappa or truncation remains an unknown force
precision limitation. It does not by itself reject geometry, while an unknown
runtime qualification keeps adoption and any one-way conclusion unknown. A
geometry result marked `inapplicable` is refused by `apply`.
