//! Surface the active ROS 2 distro as a compile-time `cfg`.
//!
//! `ROS_DISTRO` is the standard ROS 2 environment variable: a sourced
//! `setup.bash` exports it (the ros2 convention), and this repo's `nix develop`
//! shells export the same. Reading it here bakes the distro into the build as
//! `cfg(ros_distro = "<distro>")`, so distro-conditional code is selected at
//! compile time and needs nothing set at run time — e.g. the e2e harness picks
//! the `ros2 bag record` topic-list spelling that the build's distro supports.

use std::env;

fn main() {
    // Rebuild when the active distro changes (a different dev shell / sourced env).
    println!("cargo::rerun-if-env-changed=ROS_DISTRO");

    let active = env::var("ROS_DISTRO").unwrap_or_default();

    // Every value used in a `cfg(ros_distro = "…")` predicate must be declared
    // for Rust's check-cfg lint. List the distros the project targets, and add
    // the active one if it is some other (future) distro, so the build never
    // warns about an "unexpected" value it set itself.
    let mut distros = ["humble", "jazzy", "lyrical", "rolling"]
        .map(String::from)
        .to_vec();
    if !active.is_empty() && !distros.contains(&active) {
        distros.push(active.clone());
    }
    let values = distros
        .iter()
        .map(|d| format!("\"{d}\""))
        .collect::<Vec<_>>()
        .join(",");
    println!("cargo::rustc-check-cfg=cfg(ros_distro, values({values}))");

    if !active.is_empty() {
        println!("cargo::rustc-cfg=ros_distro=\"{active}\"");
    }
}
