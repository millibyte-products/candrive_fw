// Tells Cargo to put memory.x in the OUT_DIR where cortex-m-rt's link.x
// will pick it up. Standard cortex-m-rt boilerplate.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    let sha = env::var("CANDRIVE_GIT_SHA").ok().filter(|s| !s.is_empty())
        .or_else(|| Command::new("git")
            .args(["rev-parse", "--short=10", "HEAD"])
            .output().ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()))
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=CANDRIVE_GIT_SHA={sha}");
    println!("cargo:rerun-if-env-changed=CANDRIVE_GIT_SHA");
}
