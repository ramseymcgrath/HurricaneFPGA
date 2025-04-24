//! Version information.

use std::env;

/// Returns the version string for the application
pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Returns detailed version information
pub fn version_info(with_dependencies: bool) -> String {
    let output = format!("\
Hurricane build information:
  Version: {}",
        env!("CARGO_PKG_VERSION")
    );

    if with_dependencies {
        format!("{}\n\nBuilt with dependencies:\n  nusb 0.1.13\n  serialport 4.2\n  clap 4.4", output)
    } else {
        output
    }
}
