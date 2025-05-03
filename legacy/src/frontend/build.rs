//! Code to be executed at build time.

fn main() {
    // No build-time operations needed
    println!("cargo:rerun-if-changed=Cargo.toml");
}
