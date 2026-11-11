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
qualification and the backend crash receipt. `apply` creates native unit-force
loads with tabular amplitudes. When the named step already exists and is a
transient step, the loads and amplitude tables are added to that step while
its boundary conditions, integration, method, damping, output cadence and
other loads stay unchanged. When the step is absent, `apply` creates a
transient step; pass `boundary_conditions` to enable existing supports. The
reply reports the effective settings and boundary-condition count, and warns
when a newly created step has no supports. The default integration is average
acceleration, the default method is direct, and the default damping is zero.

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
damping, output cadence and solve mass are accepted for a new step and are
returned in `transient_settings` for the solver receipt. The incoming time
grid must be uniform; existing-step grids must have the same interval count
and time step (within the documented rounding tolerance), and no implicit
interpolation is performed. The time-axis intervals must agree within
`1e-12 s * max(1, |interval|)`; this is the admission tolerance for accumulated
floating-point roundoff. The step time is always computed from the complete
duration, `(last_time - first_time) / (frames - 1)`, after that check rather
than copied from the first interval. The reply exposes this source and
tolerance.

The full product regression is kept in
`crates/engine/tests/real_load_transfer.rs`. On Dell it opens the frozen
structural model, retains the historical `Contact` step as an untouched
reference, receives the 1025 point-force frames, applies the mapped native
loads to a new step with `clamp`, average acceleration, direct integration and
consistent mass, solves that step, probes the result, and writes the raw
product reply before comparing it with the frozen history. Its time-axis
comparison allows `1e-12 s` for floating-point rounding; the five historical
checks use a relative threshold of `1e-9`. The independent 3-D comparison is
then run by
`记录/第五阶段/回归/补齐候选/当前程序重算/compare_real_product.py` and retains
the three frozen windows around 0.43%, 0.43% and 0.06%.

The frozen tolerances are relative `1e-10` with absolute `1e-12` for the
conservation checks, activation-area change at most 5%, changed-component
weight at most 5%, gap change at most 2%, and peak-sampling loss at most 1%.
Missing per-frame effective kappa or truncation remains an unknown force
precision limitation. It does not by itself reject geometry, while an unknown
runtime qualification keeps adoption and any one-way conclusion unknown. A
geometry result marked `inapplicable` is refused by `apply`.
