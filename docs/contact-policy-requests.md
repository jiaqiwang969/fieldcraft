# Contact policy declarations and qualification

`model.info.contact_policy_capabilities` reports the current compiled model's ability to
represent `Step.contact_perturbation`, its selected maps and the outstanding runtime checks.
It is separate from the versioned parameter report: no existing snapshot, capability-receipt
schema or result encoding is changed. Availability is checked against the actual `Step`
serialization, not inferred from another build's documentation.

The limited declaration is a map from case-sensitive contact names to `"stick"`. A nonempty
map must cover every contact. Only explicit pairs, a frequency step, an earlier nonlinear
geometry base, and unchanged support names pass declaration inspection. `slide`, `open`,
unknown names, omissions, a linear base and general/self contact are refused. Empty maps
retain legacy selection; they do not implicitly select explicit stick. Non-perturbation
steps are reported as `not_requested`.

`fieldcraft_io::contact_policy::PolicyRequest` freezes a **proposal**, using the native field
spelling and values, against the current receiver snapshot. It does not add a model field,
apply constraints, merge a solver or introduce an alternative control input. Its schema is
`fieldcraft-contact-policy-request/1`. `freeze` checks declaration prerequisites; `validate`
checks the current snapshot and prerequisites. `digest` hashes compact serialized request
bytes. The host must freeze that digest in its trusted run lease; `decode` takes that expected
digest independently of the received JSON, bounds the request to 1 MiB and checks it against
the current model. Pretty printing does not change the semantic digest.

`require_applied` additionally compares the map with the actual serialized native step. It
refuses `NativeContactPolicyNotApplied` when a proposal has not been installed in the model.
Thus a receiver cannot accept an explicit request merely because a legacy model reopens.
On builds without the native field, the existing command form rejects the parameter by name.
The proposal JSON and native model remain distinct artifacts. Do not pass a proposal to a
solver as if it had already modified that model.

Even successful native application is **not runtime qualification**. Explicit hard stick
requires fully closed planar linear solid faces, a fixed main surface, full reference
coverage, positive guarded pressure, acceptable gap/thickness, positive static friction,
strict friction-cone margin, no slipping and zero interface increments after the actual
constraint reduction. These must come from the actual equilibrium and reduction, not input
friction alone. Pairing, normals, active set, mortar rows, terminal structural tangent and
reference mass must belong to that same qualified state. This module deliberately does not
duplicate solver eligibility calculations or parse warning text into structured facts.

The report says runtime qualification is unverified. There is no `qualified=true` field in
a request. Native warning text is not a stable state protocol; original runtime receipts and
program identities remain necessary. No verified commercial keyword mapping is claimed;
existing writer refusals must remain when the native field is integrated. Open/slide policies
and generalized moving interfaces are outside this declaration scope.

Tests in `crates/io/tests/contact_policy.rs` cover declaration/reference checks, exact proposal
binding, native archive and request-file readback, mismatched requests/models and forged
qualification metadata. The small declaration mesh is not a solved contact benchmark.
`cmd::model::contact_policy_tests` exercises the existing production command refusal and
reports actual availability. Numeric qualification requires the solver implementation and
its independent validation; neither this report nor a saved proposal supplies it.
