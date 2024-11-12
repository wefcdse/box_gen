extern crate cbindgen;

use std::env;

use cbindgen::Language;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    // let mut crate_dir = PathBuf::from(crate_dir);
    // crate_dir.push("bindings");
    cbindgen::Builder::new()
        // .with_no_includes()
        .with_crate(crate_dir)
        .with_cpp_compat(false)
        .with_language(Language::C)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("bindings.h");
}
