# The two deployable crates built reproducibly with nix, against the nix ROS
# closure (rosEnv) — a build check that mirrors the dev shell without the system
# cargo. This is NOT the deployment artifact: the target builds these natively
# against its own apt ROS2 (see README "Native build on the target"), since a
# nix-built binary bakes /nix/store RPATHs and would drag the nix closure along
# instead of using the host's ROS. Only edgestream-rec and trigger-pub are built
# — r2r-sub/rclrs-sub are out of scope. r2r's build script (bindgen + rcl codegen)
# needs the same environment the dev shell's shellHook sets: rosEnv's setup hook
# exports AMENT_PREFIX_PATH, and the explicit knobs below match the shell.
{ pkgs, rosEnv, idlPackageFilter, rosDistro, src, cargoLockFile }:

let
  mkBin = pname: pkgs.rustPlatform.buildRustPackage {
    inherit pname src;
    version = "0.0.1";
    cargoLock = {
      lockFile = cargoLockFile;
      # Git-sourced workspace dependencies need their vendor hash here. rclrs is
      # the fork used only by rclrs-sub; r2r is pinned to its 0.9.6 git tag (for
      # lyrical support — see Cargo.toml) until 0.9.6 reaches crates.io.
      outputHashes."rclrs-0.7.0" = "sha256-8wg0Qyems0XlFLh4gFUHFSAc0xaXNWY4nB+PvM36fmw=";
      outputHashes."r2r-0.9.6" = "sha256-1DQPrRQOYzxTckzyH0p6pnyEy1lOw/OmU0sDAMNzHpg=";
    };
    # Build only the deployable crate (not r2r-sub/rclrs-sub).
    cargoBuildFlags = [ "-p" pname ];
    doCheck = false;
    nativeBuildInputs = [ pkgs.clang pkgs.pkg-config rosEnv ];
    buildInputs = [ rosEnv ];
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
    IDL_PACKAGE_FILTER = idlPackageFilter;
    ROS_DISTRO = rosDistro;
  };
in {
  edgestream-rec = mkBin "edgestream-rec";
  edgestream-rec-cont = mkBin "edgestream-rec-cont";
  trigger-pub = mkBin "trigger-pub";
}
