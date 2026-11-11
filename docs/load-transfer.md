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
loads with tabular amplitudes in a linear transient structural step. The mapped
history is therefore usable by the structural solver without rebuilding loads
by hand.

The frozen tolerances are relative `1e-10` with absolute `1e-12` for the
conservation checks, activation-area change at most 5%, changed-component
weight at most 5%, gap change at most 2%, and peak-sampling loss at most 1%.
Missing per-frame effective kappa or truncation remains an unknown force
precision limitation. It does not by itself reject geometry, while an unknown
runtime qualification keeps adoption and any one-way conclusion unknown. A
geometry result marked `inapplicable` is refused by `apply`.
