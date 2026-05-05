// Common's build.rs has two jobs:
//   1. Drop common.x (our linker script) into OUT_DIR.
//   2. Tell rustc to use common.x AS THE LINKER SCRIPT and to NOT use
//      cortex-m-rt's link.x (which would inject a vector table and
//      .text.Reset_Handler that we explicitly don't want).

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("common.x"))
        .unwrap()
        .write_all(include_bytes!("common.x"))
        .unwrap();

    // Make common.x discoverable to the linker, and tell the linker to use it.
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tcommon.x");

    // Override the workspace-level config which adds `-Tlink.x` (cortex-m-rt's).
    // We can't remove it, but providing -Tcommon.x first means our MEMORY
    // declaration wins; we still need to make link.x harmless. Easiest: ship
    // an empty stand-in so linker doesn't error out if the workspace flag
    // is somehow applied. We do that by emitting an empty link.x to OUT_DIR.
    File::create(out.join("link.x"))
        .unwrap()
        .write_all(b"/* empty - real linker script is common.x */\n")
        .unwrap();

    println!("cargo:rerun-if-changed=common.x");
    println!("cargo:rerun-if-changed=build.rs");
}
