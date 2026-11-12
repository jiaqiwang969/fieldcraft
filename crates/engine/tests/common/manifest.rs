//! Helpers for testing compatibility with manifests written before derived metadata was added.

use std::{
    fs,
    io::{Cursor, Read, Write},
    path::Path,
};

/// Remove the optional identity and capability blocks from a package in place.
pub fn remove_derived_metadata(path: &Path) {
    let bytes = fs::read(path).unwrap();
    let mut source = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    let mut target = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for index in 0..source.len() {
        let mut entry = source.by_index(index).unwrap();
        let name = entry.name().unwrap().to_string();
        let mut data = Vec::new();
        entry.read_to_end(&mut data).unwrap();
        if name == "fieldcraft.json" {
            let mut manifest: serde_json::Value = serde_json::from_slice(&data).unwrap();
            let object = manifest.as_object_mut().unwrap();
            object.remove("identity");
            object.remove("capabilities");
            data = serde_json::to_vec(&manifest).unwrap();
        }
        target.start_file(name, zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored)).unwrap();
        target.write_all(&data).unwrap();
    }
    fs::write(path, target.finish().unwrap().into_inner()).unwrap();
}
