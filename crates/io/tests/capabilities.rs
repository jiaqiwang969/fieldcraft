use fieldcraft_io::capabilities::*;
use fieldcraft_model::{Contact, ContactSurfaces, Friction, Model};
use serde_json::json;

fn model() -> Model {
    let mut m = Model::default();
    m.add_material("solid", serde_json::from_value(json!({"elastic":{"type":"isotropic","young":210e9,"poisson":0.3},"density":7800.0})).unwrap())
        .unwrap();
    let mut c = Contact::new(ContactSurfaces::General { surface: None });
    c.friction = Friction::coulomb(0.25);
    c.penalty_scale = 0.5;
    m.add_contact("pair", c).unwrap();
    m
}
fn unknown_controls() -> ContactControls {
    ContactControls {
        scheme: CONTACT_SCHEME.into(),
        d_hat_m: 0.001,
        kappa_requested_pa: 1e9,
        kappa_effective_pa: Observation::Unknown { reason: "no backend observation".into() },
        kappa_clamped: Observation::Unknown { reason: "no clipping receipt".into() },
    }
}
#[test]
fn physical_numerical_mapping_and_archive_readback() {
    let m = model();
    let r = report(&m).unwrap();
    assert_eq!(isotropic_parameters(&m, "solid").unwrap(), IsotropicParameters { young_pa: 210e9, poisson: 0.3, density_kg_m3: 7800.0 });
    assert_eq!(r.contacts["pair"].physical["friction_static"], 0.25);
    assert_eq!(r.contacts["pair"].numerical["penalty_scale"], 0.5);
    assert!(r.contacts["pair"].physical.get("penalty_scale").is_none());
    assert!(r.contacts["pair"].numerical.get("kappa_effective_pa").is_none());
    let bytes = fieldcraft_io::write(&m, &Default::default(), &[], &Default::default(), &Default::default()).unwrap();
    let opened = fieldcraft_io::read(&bytes).unwrap();
    assert_eq!(r, report(&opened.model).unwrap());
    let serialized = serde_json::to_vec(&r).unwrap();
    let restored: Report = serde_json::from_slice(&serialized).unwrap();
    restored.validate(&opened.model).unwrap();
    println!("capabilities={}", String::from_utf8(serialized).unwrap());
}
#[test]
fn unsupported_physics_is_retained_and_named() {
    let mut m = model();
    let mut mat = m.material("solid").unwrap().clone();
    mat.hyperelastic = Some(fieldcraft_model::Hyperelastic::NeoHookean {});
    m.replace_material("solid", mat).unwrap();
    assert!(isotropic_parameters(&m, "solid").unwrap_err().to_string().contains("solid.hyperelastic"));
    let r = report(&m).unwrap();
    assert_eq!(r.materials["solid"].physical["hyperelastic"]["type"], "neo_hookean");
    assert!(matches!(r.materials["solid"].isotropic_extraction, Extraction::Refused { .. }));
    assert!(isotropic_parameters(&m, "missing").unwrap_err().to_string().contains("UnknownMaterial"));
    let mut mat = m.material("solid").unwrap().clone();
    mat.hyperelastic = None;
    mat.density = None;
    m.replace_material("solid", mat).unwrap();
    assert!(isotropic_parameters(&m, "solid").unwrap_err().to_string().contains("MissingDensity"));
}
#[test]
fn stale_and_forged_capability_report_rejected() {
    let mut m = model();
    let r = report(&m).unwrap();
    let mut c = m.contact("pair").unwrap().clone();
    c.penalty_scale = 0.25;
    m.replace_contact("pair", c).unwrap();
    assert!(r.validate(&m).unwrap_err().to_string().contains("CapabilityModelMismatch"));
    let mut forged = report(&m).unwrap();
    forged.scheme = "unknown/2".into();
    assert!(forged.validate(&m).unwrap_err().to_string().contains("UnsupportedCapabilityScheme"));
}
#[test]
fn unknown_effective_values_roundtrip_without_defaults_and_refuse_quantitative_use() {
    let c = unknown_controls();
    persisted_controls(&c, false, "controls-unknown.json");
    let bytes = c.encode(false).unwrap();
    let restored = ContactControls::decode(&bytes, false).unwrap();
    assert_eq!(c, restored);
    assert!(ContactControls::decode(&bytes, true).unwrap_err().to_string().contains("MissingEffectiveKappa"));
    for key in ["kappa_effective_pa", "kappa_clamped"] {
        let mut missing = serde_json::to_value(&c).unwrap();
        missing.as_object_mut().unwrap().remove(key);
        assert!(ContactControls::decode(&serde_json::to_vec(&missing).unwrap(), false).is_err());
    }
    let mut unknown = c;
    unknown.kappa_effective_pa = Observation::Unknown { reason: String::new() };
    assert!(unknown.encode(false).is_err());
}
#[test]
fn sourced_controls_preserve_requested_effective_and_clipping_separately() {
    let mut c = unknown_controls();
    let sha = fieldcraft_io::identity::artifact_digest(b"synthetic backend observation");
    c.kappa_effective_pa = Observation::Known { value: 5e8, source_sha256: sha.clone(), locator: "effective/kappa".into() };
    assert!(c.validate(true).unwrap_err().to_string().contains("MissingKappaClamped"));
    c.kappa_clamped = Observation::Known { value: true, source_sha256: sha, locator: "effective/clamped".into() };
    persisted_controls(&c, true, "controls-synthetic-known.json");
    assert_eq!(ContactControls::decode(&c.encode(true).unwrap(), true).unwrap(), c);
    assert_eq!(c.kappa_requested_pa, 1e9);
    c.kappa_effective_pa = Observation::Known { value: 5e8, source_sha256: "not-a-digest".into(), locator: "effective/kappa".into() };
    assert!(c.validate(true).unwrap_err().to_string().contains("MissingObservationSource"));
    c = unknown_controls();
    c.d_hat_m = f64::NAN;
    assert!(c.encode(false).is_err());
    c = unknown_controls();
    c.kappa_requested_pa = -1.0;
    assert!(c.encode(false).is_err());
}
#[test]
fn tampered_archive_capabilities_are_rejected() {
    use std::io::{Cursor, Read, Write};
    let m = model();
    let bytes = fieldcraft_io::write(&m, &Default::default(), &[], &Default::default(), &Default::default()).unwrap();
    let mut src = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    let mut dst = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for i in 0..src.len() {
        let mut file = src.by_index(i).unwrap();
        let name = file.name().unwrap().to_string();
        let mut data = Vec::new();
        file.read_to_end(&mut data).unwrap();
        if name == "fieldcraft.json" {
            let mut j: serde_json::Value = serde_json::from_slice(&data).unwrap();
            j["capabilities"] = serde_json::to_value(report(&m).unwrap()).unwrap();
            j["capabilities"]["contacts"]["pair"]["numerical"]["penalty_scale"] = json!(1.0);
            data = serde_json::to_vec(&j).unwrap();
        }
        dst.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
        dst.write_all(&data).unwrap();
    }
    assert!(fieldcraft_io::read(&dst.finish().unwrap().into_inner()).err().unwrap().to_string().contains("CapabilityModelMismatch"));
}

#[test]
fn receipt_version_size_and_orthotropic_mapping_are_explicit() {
    let mut c = unknown_controls();
    c.scheme = "other/2".into();
    assert!(c.encode(false).unwrap_err().to_string().contains("UnsupportedContactControlsScheme"));
    c = unknown_controls();
    c.kappa_clamped = Observation::Unknown { reason: "x".repeat(65536) };
    assert!(c.encode(false).unwrap_err().to_string().contains("ContactControlsTooLarge"));
    assert!(ContactControls::decode(&vec![b' '; 65537], false).unwrap_err().to_string().contains("ContactControlsTooLarge"));
    let mut m = model();
    let mut material = m.material("solid").unwrap().clone();
    material.elastic = Some(fieldcraft_model::Elastic::EngineeringConstants { young: [1000.0; 3], poisson: [0.2; 3], shear: [400.0; 3] });
    m.replace_material("solid", material).unwrap();
    assert!(isotropic_parameters(&m, "solid").unwrap_err().to_string().contains("UnsupportedElasticMapping"));
    let bytes = fieldcraft_io::write(&m, &Default::default(), &[], &Default::default(), &Default::default()).unwrap();
    let opened = fieldcraft_io::read(&bytes).unwrap();
    assert_eq!(report(&m).unwrap(), report(&opened.model).unwrap());
}

fn persisted_controls(c: &ContactControls, require: bool, name: &str) {
    let dir = std::env::var_os("FIELDCRAFT_CAPABILITY_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-capability-receipts-{}", std::process::id())));
    fieldcraft_io::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    let bytes = c.encode(require).unwrap();
    fieldcraft_io::fs::write_atomic(&path, &bytes, false).unwrap();
    let restored = fieldcraft_io::fs::read_capped(&path, 65536).unwrap();
    assert_eq!(restored, bytes);
    assert_eq!(&ContactControls::decode(&restored, require).unwrap(), c);
}

#[test]
fn observation_digest_shape_and_locator_refusals_survive_readback() {
    for (sha, accepted) in [
        ("0123456789abcdef".repeat(4), true),
        ("0".repeat(64), true),
        ("0".repeat(63), false),
        ("0".repeat(65), false),
        ("A".repeat(64), false),
        ("g".repeat(64), false),
        ("é".repeat(32), false),
    ] {
        let mut c = unknown_controls();
        c.kappa_clamped = Observation::Known { value: false, source_sha256: sha, locator: "initialization/clamped".into() };
        if accepted {
            assert_eq!(ContactControls::decode(&c.encode(false).unwrap(), false).unwrap(), c);
            if let Observation::Known { locator, .. } = &mut c.kappa_clamped {
                *locator = " \n".into();
            }
        }
        assert!(c.encode(false).unwrap_err().to_string().contains("MissingObservationSource"));
    }
}
