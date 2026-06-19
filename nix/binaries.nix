# The Rust binaries built reproducibly with nix, against the nix ROS closure
# (rosEnv) — a build check that mirrors the dev shell without the system cargo.
# This is NOT the deployment artifact: the target builds these natively against
# its own apt ROS2 (see README "Native build on the target"), since a nix-built
# binary bakes /nix/store RPATHs and would drag the nix closure along instead of
# using the host's ROS. Both are built: clipper (the deployable recorder) and
# trigger-pub (the example trigger publisher, examples/trigger-pub). r2r's build
# script (bindgen + rcl codegen) needs the same environment the dev shell's
# shellHook sets: rosEnv's setup hook exports AMENT_PREFIX_PATH, and the explicit
# knobs below match the shell.
{ pkgs, rosEnv, idlPackageFilter, rosDistro, src, cargoLockFile }:

let
  mkBin = { pname, cargoPkg ? pname }: pkgs.rustPlatform.buildRustPackage {
    inherit pname src;
    version = "0.0.1";
    cargoLock = {
      lockFile = cargoLockFile;
      # r2r is a git-sourced workspace dependency pinned to its 0.9.6 git tag
      # (for lyrical support — see Cargo.toml) until 0.9.6 reaches crates.io, so
      # it needs its vendor hash here.
      outputHashes."r2r-0.9.6" = "sha256-1DQPrRQOYzxTckzyH0p6pnyEy1lOw/OmU0sDAMNzHpg=";
    };
    # Build only the named crate (`-p` is independent of its directory).
    cargoBuildFlags = [ "-p" cargoPkg ];
    doCheck = false;
    nativeBuildInputs = [ pkgs.clang pkgs.pkg-config rosEnv ];
    buildInputs = [ rosEnv ];
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
    IDL_PACKAGE_FILTER = idlPackageFilter;
    ROS_DISTRO = rosDistro;
  };
in {
  clipper = mkBin { pname = "clipper";  };
  # The example trigger publisher (examples/trigger-pub), built here too as a check.
  trigger-pub = mkBin { pname = "trigger-pub"; };
}
