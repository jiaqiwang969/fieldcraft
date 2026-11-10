use fieldcraft_model::Severity;
use fieldcraft_model::units::{DisplaySystem, Preset, Quantity};
use serde_json::{Value, json};

use super::{listed, name};
use crate::requirement::Requirement;
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};

pub const INFO: CommandSpec =
    CommandSpec { id: "model.info", label: "Model Info", params: "{}", form: Some(&[]), kind: CommandKind::Model, requires: &[], run: info };

pub const NODE_COORDS: CommandSpec = CommandSpec {
    id: "model.node.coordinates",
    label: "Node Coordinates",
    params: r#"{"nodes": [1, 2]} — returns the requested mesh node labels and coordinates in the model's display length units"#,
    form: Some(super::forms::NODE_COORDS),
    kind: CommandKind::Model,
    requires: &[Requirement::Mesh],
    run: node_coordinates,
};

pub const UNITS: CommandSpec = CommandSpec {
    id: "model.units",
    label: "Unit System",
    params: r#"{"system": "mm-t-s-N-MPa|m-kg-s-N-Pa|mm-kg-ms-kN-GPa|mm-g-ms-N-MPa|in-lbf-s-psi|default", "temperature"?: "°C|K|°F" (temperature differences and per-degree units follow it), "overrides": {"energy": "J", "angle": "deg", …}} — the units this model is shown and typed in (bare numbers of later commands are in them); `default` follows the user's setting. Stored values never change: every quantity is stored in SI. `unspecified` (older models) reads as mm-t-s-N-MPa"#,
    form: Some(super::forms::UNITS),
    kind: CommandKind::Model,
    requires: &[],
    run: units,
};

pub const CHECK: CommandSpec =
    CommandSpec { id: "model.check", label: "Check Model", params: "{}", form: Some(&[]), kind: CommandKind::Model, requires: &[], run: check };

pub const SET_RESOLVE: CommandSpec = CommandSpec {
    id: "model.set.resolve",
    label: "Resolve Set",
    params: r#"{"name": "root"} — the set's nodes, elements and element faces on the current mesh (the first 100 of each); fails naming every member that is not on the mesh"#,
    form: Some(super::forms::SET_RESOLVE),
    kind: CommandKind::Model,
    requires: &[Requirement::Set],
    run: set_resolve,
};

fn info(s: &mut Session, _: &Value) -> Result<Value> {
    let counts: serde_json::Map<String, Value> = s.model.element_counts().into_iter().map(|(k, n)| (k.name().to_string(), json!(n))).collect();
    let definitions: serde_json::Map<String, Value> = s.model.definition_counts().into_iter().map(|(k, n)| (k.to_string(), json!(n))).collect();
    Ok(json!({
        "nodes": s.model.nodes().len(),
        "elements": counts,
        "display_units": s.units().label(),
        "own_units": s.model.display_units().is_some(),
        "definitions": definitions,
        "parameter_capabilities": fieldcraft_io::capabilities::report(&s.model)?,
        "contact_policy_capabilities": fieldcraft_io::contact_policy::report(&s.model)?,
    }))
}

fn node_coordinates(s: &mut Session, p: &Value) -> Result<Value> {
    let nodes = p.get("nodes").and_then(Value::as_array).ok_or_else(|| EngineError::Params("`nodes` must be an array".into()))?;
    if nodes.len() > 10_000 {
        return Err(EngineError::Params("`nodes` may contain at most 10000 labels".into()));
    }
    let mut out = Vec::with_capacity(nodes.len());
    for value in nodes {
        let label =
            value.as_u64().and_then(|n| u32::try_from(n).ok()).ok_or_else(|| EngineError::Params("node labels must be whole numbers".into()))?;
        let node = s.model.node(label).ok_or_else(|| EngineError::Params(format!("node {label} is not on the mesh")))?;
        let xyz = node.xyz.map(|value| s.units().express(Quantity::Length, value).unwrap_or(f64::NAN));
        out.push(json!({"node": label, "xyz": xyz}));
    }
    Ok(json!({"nodes": out, "units": s.units().symbol(fieldcraft_model::units::Quantity::Length)}))
}

/// The units a display system shows the common quantities in, for replies.
pub(crate) fn unit_table(d: &DisplaySystem) -> Value {
    let q = [
        Quantity::Length,
        Quantity::Mass,
        Quantity::Time,
        Quantity::Force,
        Quantity::Pressure,
        Quantity::Density,
        Quantity::Energy,
        Quantity::Temperature,
        Quantity::TemperatureDifference,
        Quantity::ThermalExpansion,
        Quantity::Angle,
    ];
    Value::Object(q.iter().map(|q| (q.name().to_string(), json!(d.symbol(*q)))).collect())
}

fn units(s: &mut Session, p: &Value) -> Result<Value> {
    let label = p.get("system").and_then(Value::as_str).unwrap_or_default();
    let base = match label {
        "default" => None,
        _ => Some(Preset::from_label(label).ok_or_else(|| {
            let known: Vec<&str> = Preset::ALL.iter().map(|u| u.label()).collect();
            EngineError::Params(format!("`system` must be one of {} or default", known.join(", ")))
        })?),
    };
    let overrides = p.get("overrides").and_then(Value::as_object);
    if base.is_none() && overrides.is_some_and(|o| !o.is_empty()) {
        return Err(EngineError::invalid("overrides", "need a `system` they change"));
    }
    let mut system = base.map(DisplaySystem::new);
    if let (Some(d), Some(o)) = (system.as_mut(), overrides) {
        for (name, unit) in o {
            let q = Quantity::from_name(name).ok_or_else(|| EngineError::invalid(format!("overrides/{name}"), "is not a quantity"))?;
            let unit = unit.as_str().ok_or_else(|| EngineError::invalid(format!("overrides/{name}"), "must be a unit, such as \"J\""))?;
            *d = d.clone().with_override(q, unit).map_err(|e| EngineError::invalid(format!("overrides/{name}"), e.to_string()))?;
        }
    }
    // A temperature scale of its own (the form's choice): the temperature differences and the
    // per-degree quantities follow it (`DisplaySystem`), unless overridden too.
    if let Some(t) = p.get("temperature").filter(|t| !t.is_null()) {
        let t = t.as_str().filter(|t| ["°C", "K", "°F"].contains(t)).ok_or_else(|| EngineError::invalid("temperature", "must be °C, K or °F"))?;
        if overrides.is_some_and(|o| o.contains_key("temperature")) {
            return Err(EngineError::invalid("temperature", "is given twice (also in overrides)"));
        }
        let d = system.as_mut().ok_or_else(|| EngineError::invalid("temperature", "needs a `system` it changes"))?;
        *d = d.clone().with_override(Quantity::Temperature, t).map_err(|e| EngineError::invalid("temperature", e.to_string()))?;
    }
    s.model.set_display_units(system);
    Ok(json!({"system": s.units().label(), "own_units": s.model.display_units().is_some(), "units": unit_table(s.units())}))
}

fn check(s: &mut Session, _: &Value) -> Result<Value> {
    let mut diagnostics = s.model.check();
    if let Some(design) = s.design.as_deref() {
        for (body, n) in super::replace::stale(&s.model, design) {
            diagnostics.push(fieldcraft_model::Diagnostic {
                severity: Severity::Warning,
                subject: format!("body `{body}`"),
                message: format!(
                    "{n} elements mesh a body the open design no longer has (its partitions changed since it was meshed); mesh.clear removes them"
                ),
            });
        }
    }
    if let Some(why) = s.mesh_outdated() {
        diagnostics.push(fieldcraft_model::Diagnostic { severity: Severity::Warning, subject: "mesh".into(), message: why });
    }
    let errors = diagnostics.iter().filter(|d| d.severity == Severity::Error).count();
    let first_error = diagnostics.iter().find(|d| d.severity == Severity::Error).map(ToString::to_string);
    let progress = crate::progress::to_json(&s.progress_given(first_error));
    Ok(json!({
        "ready": errors == 0,
        "errors": errors,
        "warnings": diagnostics.len() - errors,
        "diagnostics": diagnostics,
        "progress": progress,
    }))
}

fn set_resolve(s: &mut Session, p: &Value) -> Result<Value> {
    let r = s.model.resolve_set(name(p)?)?;
    Ok(json!({"nodes": listed(&r.nodes), "elements": listed(&r.elements), "faces": listed(&r.faces)}))
}

#[cfg(test)]
mod temperature_tests {
    use serde_json::json;

    use crate::Session;

    /// A temperature scale of its own carries to ΔT and the per-degree units; numbers typed
    /// after follow it (a 1.8 °F rise is 1 K).
    #[test]
    fn a_temperature_scale_carries_to_differences_and_per_degree_units() {
        let mut s = Session::default();
        let r = s.execute("model.units", &json!({"system": "mm-t-s-N-MPa", "temperature": "°F"})).unwrap();
        assert_eq!(
            (&r["units"]["temperature"], &r["units"]["temperature_difference"], &r["units"]["thermal_expansion"], &r["units"]["pressure"]),
            (&json!("°F"), &json!("°F"), &json!("°F⁻¹"), &json!("MPa")),
            "{r}"
        );
        s.execute("model.material.create", &json!({"name": "Steel", "thermal": {"expansion": 1e-5, "reference_temperature": 68}})).unwrap();
        let t = s.model.material("Steel").unwrap().thermal.unwrap();
        assert_eq!(t.reference_temperature, 293.15);
        assert!((t.expansion.unwrap().axes()[0] - 1.8e-5).abs() < 1e-20, "{t:?}");
        for bad in [
            json!({"system": "mm-t-s-N-MPa", "temperature": "°R"}),
            json!({"system": "mm-t-s-N-MPa", "temperature": "°F", "overrides": {"temperature": "K"}}),
        ] {
            assert!(s.execute("model.units", &bad).is_err(), "{bad}");
        }
    }
}

#[cfg(test)]
mod capability_tests {
    use crate::Session;
    use serde_json::json;
    #[test]
    fn parameter_capabilities_command_save_reopen_and_refusal() {
        let mut s = Session::default();
        s.execute("model.units", &json!({"system":"m-kg-s-N-Pa"})).unwrap();
        s.execute("model.material.create", &json!({"name":"Solid","elastic":{"type":"isotropic","young":210e9,"poisson":0.3},"density":7800.0}))
            .unwrap();
        s.execute("interaction.contact.general", &json!({"name":"Contact","friction":{"static":0.25},"penalty_scale":0.5})).unwrap();
        let before = s.execute("model.info", &json!({})).unwrap()["parameter_capabilities"].clone();
        assert_eq!(before["materials"]["Solid"]["isotropic_extraction"]["parameters"]["young_pa"], 210e9);
        assert_eq!(before["contacts"]["Contact"]["numerical"]["penalty_scale"], 0.5);
        let dir = std::env::var_os("FIELDCRAFT_CAPABILITY_EVIDENCE")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-capabilities-{}", std::process::id())));
        fieldcraft_io::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("parameters.fieldcraft");
        s.execute("file.save", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        assert_eq!(reopened.execute("model.info", &json!({})).unwrap()["parameter_capabilities"], before);
        assert!(reopened.execute("interaction.contact.edit", &json!({"name":"Contact","penalty_scale":2.0})).is_err());
        assert_eq!(reopened.execute("model.info", &json!({})).unwrap()["parameter_capabilities"], before);
        fieldcraft_io::fs::write_atomic(&dir.join("capabilities.json"), &serde_json::to_vec_pretty(&before).unwrap(), false).unwrap();
        println!("PARAMETER_CAPABILITIES {before}");
    }
}

#[cfg(test)]
mod contact_policy_tests {
    use crate::Session;
    use fieldcraft_model::{BoundaryCondition, Contact, ContactSurfaces, ElementKind, Friction, Model, NonlinearStatic, Procedure, Step, Target};
    use serde_json::json;
    fn policy_fixture() -> Model {
        let mut m = Model::default();
        for (i, xyz) in
            [[0., 0., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., -1.], [0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].into_iter().enumerate()
        {
            m.set_node(i as u32 + 1, xyz).unwrap();
        }
        m.set_element(10, ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
        m.set_element(20, ElementKind::Tet4, vec![5, 6, 7, 8]).unwrap();
        for (name, element) in [("main", 10), ("secondary", 20)] {
            m.add_set(name, serde_json::from_value(json!({"faces":[{"element":element,"face":1}]})).unwrap()).unwrap();
        }
        m.add_set("fixed", serde_json::from_value(json!({"nodes":[1,2,3,4]})).unwrap()).unwrap();
        m.add_boundary_condition("Fixed", BoundaryCondition::Fixed { target: Target::Set("fixed".into()) }).unwrap();
        let mut contact = Contact::new(ContactSurfaces::Pair { main: "main".into(), secondary: "secondary".into() });
        contact.friction = Friction::coulomb(0.3);
        m.add_contact("Interface", contact).unwrap();
        let mut base = Step::new("Preload", Procedure::NonlinearStatic(NonlinearStatic::default()));
        base.boundary_conditions.insert("Fixed".into());
        m.add_step(base, None).unwrap();
        let mut modes = Step::new("Modes", Procedure::Frequency { modes: 1, min_hz: None, max_hz: None });
        modes.base_state = Some("Preload".into());
        modes.boundary_conditions.insert("Fixed".into());
        m.add_step(modes, None).unwrap();
        m
    }
    #[test]
    fn unsupported_policy_value_is_refused_with_valid_contact_and_frequency_base() {
        let mut s = Session { model: policy_fixture(), ..Default::default() };
        s.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{"Interface":"open"}})).unwrap();
        let before = s.model.clone();
        let error =
            s.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{"Interface":"coulomb_sliding"}})).unwrap_err().to_string();
        assert!(error.contains("coulomb_sliding"), "{error}");
        assert!(error.contains("unknown variant") || error.contains("UnsupportedContactPolicy"), "{error}");
        assert!(!error.contains("UnknownPolicyContact"), "{error}");
        assert_eq!(s.model, before);
        println!("UNSUPPORTED_POLICY_VALUE {error}");
    }
    #[test]
    fn contact_policy_input_is_not_silently_lost() {
        let mut s = Session::default();
        let input = json!({"name":"Trial","procedure":{"type":"static"},"contact_perturbation":{"Missing":"stick"}});
        let error = s.execute("model.step.create", &input).unwrap_err();
        assert!(error.to_string().contains("contact_perturbation"));
        assert!(s.model.step("Trial").is_none());
        println!("POLICY_REFUSED input={input} error={error}");
        s.execute("model.step.create", &json!({"name":"Legacy","procedure":{"type":"static"}})).unwrap();
        let before = s.model.clone();
        for policy in [json!({"Missing":"stick"}), json!({"Missing":"slide"}), json!(null)] {
            assert!(s.execute("model.step.edit", &json!({"name":"Legacy","contact_perturbation":policy})).is_err());
            assert_eq!(s.model, before);
        }
        let empty = s.execute("model.step.edit", &json!({"name":"Legacy","contact_perturbation":{}}));
        if fieldcraft_io::contact_policy::native_field_available() {
            empty.unwrap();
        } else {
            assert!(empty.is_err());
        }
        let info = s.execute("model.info", &json!({})).unwrap();
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Legacy"]["selection"], "not_requested");
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Legacy"]["runtime_qualification"], "unverified");
        let dir = std::env::var_os("FIELDCRAFT_POLICY_EVIDENCE")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-policy-command-{}", std::process::id())));
        fieldcraft_io::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("command-model.fieldcraft");
        s.execute("file.save", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        let after = reopened.execute("model.info", &json!({})).unwrap();
        assert_eq!(after["contact_policy_capabilities"], info["contact_policy_capabilities"]);
        fieldcraft_io::fs::write_atomic(
            &dir.join("command-policy-report.json"),
            &serde_json::to_vec_pretty(&after["contact_policy_capabilities"]).unwrap(),
            false,
        )
        .unwrap();
    }
}
