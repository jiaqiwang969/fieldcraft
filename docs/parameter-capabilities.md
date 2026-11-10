# Parameter capabilities

`model.info.parameter_capabilities` reports the current model's SI physical parameters
separately from contact numerical controls. This is an extraction/receipt capability table,
not a claim that every listed material or contact can be solved by every backend.

`fieldcraft_io::capabilities::report` is the single implementation used by the command and
native archive writer. Format-3 manifests keep the report in optional `capabilities` metadata.
The production reader recomputes it from the restored model and refuses stale or forged
reports (`CapabilityModelMismatch`). Older files without the field remain readable. As with
snapshot identity, older readers that reject unknown manifest fields may reject new files.
No result encoding or kind is changed.

| Entry | Actual operation | Boundary and evidence |
| --- | --- | --- |
| Material physical fields | Copies the complete native material definition in SI | Every field retained, including physics not mapped by the scalar extractor; `unsupported_physics_is_retained_and_named` |
| Isotropic parameters | `isotropic_parameters` extracts constant E (Pa), nu (dimensionless), density (kg/m³) | Only elastic isotropic + density; extra material fields, non-isotropic elasticity or missing density are named refusals. This does not convert to another constitutive law. |
| Contact physical fields | Static/kinetic friction and decay (s/m) from the native contact definition | Kinetic coefficient follows the native model's explicit fallback rule; not an observation of sliding or an external backend friction validation. |
| Contact numerical fields | Native penalty_scale and initial_penetration policy | Dimensionless scale/algorithm choice; never renamed to external kappa or d_hat. No new perturbation keyword mapping. |
| External numerical receipt | `ContactControls::encode/decode/validate` | Explicit positive finite d_hat (m), requested kappa (Pa), independently observed effective kappa (Pa) and clipped flag. Synthetic receipt tests verify handling, not backend physics. |

Contact receipt scheme is `fieldcraft-contact-controls/1`; unknown or missing versions are
refused. Encoded and decoded receipts are limited to 65536 bytes.

`Observation<T>` is either `known { value, source_sha256, locator }` or
`unknown { reason }`. There are no implicit defaults for missing observations. Known facts
require a well-formed SHA-256 and nonempty source locator. The host must verify the source
bytes, bind them to run/epoch/configuration and check that the locator supports the fact;
structural validation alone cannot authenticate evidence. Requested kappa is never copied
into effective kappa, and no warning never becomes `clamped=false`.

Diagnostic receipt encoding/decoding (`require_effective=false`) preserves unknowns. A
quantitative caller must explicitly pass `true` and gets `MissingEffectiveKappa` or
`MissingKappaClamped` until both are known with provenance. The receiving runtime must freeze
this requirement in its profile and cannot downgrade it after a failure. These receipts are
separate validated JSON; they are not automatically attached to a native solver result or
included in its identity. The host must bind receipt bytes in the external run contract.

No second material conversion implementation is introduced. Material constitutive mapping
requires a separately versioned conversion contract. Effective kappa/clipping observations
and the new contact perturbation policy remain pending external dependencies; they are shown
as such in every report instead of being synthesized.

Tests: `crates/io/tests/capabilities.rs` exercises parameter mapping, named refusal, receipt
round trips, malformed/missing source and non-finite controls, report tampering and archive
readback. `cmd::model::capability_tests` executes material/contact commands, `model.info`,
`file.save`, `file.open` and a refused contact edit, checking unchanged effective parameters.
