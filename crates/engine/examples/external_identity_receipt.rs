//! Decode a provider-produced binding without claiming to recompute its model identity.
use fieldcraft_engine::external::receive::ModelBinding;
use std::io::Read;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("expected binding JSON path")?;
    let mut bytes = Vec::new();
    std::fs::File::open(path)?.take(64 * 1024 + 1).read_to_end(&mut bytes)?;
    if bytes.len() > 64 * 1024 {
        return Err("binding exceeds byte cap".into());
    }
    let binding: ModelBinding = serde_json::from_slice(&bytes)?;
    binding.validate_shape()?;
    println!(
        "{}",
        serde_json::json!({
            "binding_sha256": fieldcraft_io::sha256_hex(&bytes),
            "binding": binding,
            "wire_shape_accepted": true,
            "model_identity_recomputed": false,
            "committed": false
        })
    );
    Ok(())
}
