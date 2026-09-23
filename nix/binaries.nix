# The Rust binaries built reproducibly with nix, against the nix ROS closure
# (rosEnv) — a build check that mirrors the dev shell without the system cargo.
# This is NOT the deployment artifact: the target builds these natively against
# its own apt ROS2 (see README "Native build on the target"), since a nix-built
# binary bakes /nix/store RPATHs and would drag the nix closure along instead of
# using the host's ROS. The binary that *is* a shippable nix artefact is the
# ROS-free one, which links none of this and is built once for no distro (see
# ./clipper-ros-free.nix). Both packages here are built: clipper (the deployable
# recorder, whose modes are subcommands — `clipper tail`) and trigger-pub (the
# example trigger publisher, examples/trigger-pub). r2r's build
# script (bindgen + rcl codegen) needs the same environment the dev shell's
# shellHook sets: rosEnv's setup hook exports AMENT_PREFIX_PATH, and the explicit
# knobs below match the shell.
{ pkgs, rosEnv, idlPackageFilter, rosDistro, src, cargoLockFile, version, cargoOutputHashes }:

let
  mkBin = { pname, cargoPkg ? pname, cargoFeatures ? [ ] }: pkgs.rustPlatform.buildRustPackage {
    inherit pname src version;
    cargoLock = {
      lockFile = cargoLockFile;
      outputHashes = cargoOutputHashes;
    };
    # Build only the named crate (`-p` is independent of its directory), with
    # whatever cargo features that crate needs to be the ROS-linking build.
    cargoBuildFlags = [ "-p" cargoPkg ]
      ++ pkgs.lib.optionals (cargoFeatures != [ ]) [ "--features" (pkgs.lib.concatStringsSep "," cargoFeatures) ];
    doCheck = false;
    nativeBuildInputs = [ pkgs.clang pkgs.pkg-config rosEnv ];
    buildInputs = [ rosEnv ];
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
    IDL_PACKAGE_FILTER = idlPackageFilter;
    ROS_DISTRO = rosDistro;
  };
in {
  # The recorder: cargo package and binary are both `clipper`. `ros` is what
  # makes it the device build — the feature is off by default, and without it the
  # binary would link none of the ROS closure this derivation exists to build
  # against.
  clipper = mkBin { pname = "clipper"; cargoFeatures = [ "ros" ]; };
  # The example trigger publisher (examples/trigger-pub), built here too as a check.
  trigger-pub = mkBin { pname = "trigger-pub"; };
}
