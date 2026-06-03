# Deployable binaries built reproducibly with nix. The dev shell uses system
# cargo; for a portable aarch64 image we build them here instead. Only
# edgestream-rec and trigger-pub are packaged — r2r-sub/rclrs-sub are out of
# scope. r2r's build script (bindgen + rcl codegen) needs the same environment
# the dev shell's shellHook sets: rosEnv's setup hook exports AMENT_PREFIX_PATH,
# and the explicit knobs below match the shell.
{ pkgs, rosEnv, idlPackageFilter, src, cargoLockFile }:

let
  mkBin = pname: pkgs.rustPlatform.buildRustPackage {
    inherit pname src;
    version = "0.0.1";
    cargoLock = {
      lockFile = cargoLockFile;
      # rclrs comes from a git fork used only by rclrs-sub, but the workspace
      # lock references it so the vendor step needs its hash.
      outputHashes."rclrs-0.7.0" = "sha256-8wg0Qyems0XlFLh4gFUHFSAc0xaXNWY4nB+PvM36fmw=";
    };
    # Build only the deployable crate (not r2r-sub/rclrs-sub).
    cargoBuildFlags = [ "-p" pname ];
    doCheck = false;
    nativeBuildInputs = [ pkgs.clang pkgs.pkg-config rosEnv ];
    buildInputs = [ rosEnv ];
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
    IDL_PACKAGE_FILTER = idlPackageFilter;
    ROS_DISTRO = "jazzy";
  };
in {
  edgestream-rec = mkBin "edgestream-rec";
  trigger-pub = mkBin "trigger-pub";
}
