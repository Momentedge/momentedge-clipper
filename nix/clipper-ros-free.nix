# The ROS-free `clipper` binary as a nix package — a shippable artefact rather
# than a build check, for anywhere with no ROS 2 installation: CI, and a cloud
# deployment that is handed recordings and returns clips instead of joining a
# live ROS graph.
#
# It is the complement of ./binaries.nix. Every package there is anchored to one
# ROS 2 distro and built against the nix ROS closure, which is exactly what keeps
# those from being deployable: a binary linked against that closure bakes
# /nix/store RPATHs and loads it instead of the target's own apt ROS 2. Here the
# `ros` cargo feature is simply off, so the binary links no ROS at all — there is
# no distro to select, no rosEnv, no IDL_PACKAGE_FILTER, and nothing r2r's build
# script would need. What is left is a plain rustPlatform build whose output
# depends on nothing but its own closure.
#
# The vendored hash for the git-sourced r2r is still required. Nix vendors the
# whole lockfile before cargo runs and cannot know which entries a feature
# selection will reach, and Cargo.lock carries r2r's git source whether or not
# any feature enables it — so the crate is fetched and hashed here, and never
# compiled.
{
  pkgs,
  src,
  cargoLockFile,
  version,
  cargoOutputHashes,
}:
pkgs.rustPlatform.buildRustPackage {
  pname = "clipper-ros-free";
  inherit src version;

  cargoLock = {
    lockFile = cargoLockFile;
    outputHashes = cargoOutputHashes;
  };

  # `-p clipper` with no `--features`: the crate's default feature set is the
  # ROS-free build. Naming the package also keeps the workspace's other members
  # out of the build entirely — trigger-pub links r2r unconditionally, and
  # cu-mcap-record drags the cu29 tree in.
  cargoBuildFlags = ["-p" "clipper"];

  # The crate's unit tests belong to CI's ROS-free lane, which runs them on a
  # stock toolchain far more cheaply than a release-profile rebuild in the
  # sandbox would. What this derivation checks instead is the property that makes
  # it an artefact at all — see installCheckPhase.
  doCheck = false;

  # The build sandbox contains no ROS 2 of any kind, so running the binary here
  # is the proof that it needs none: were the `ros` feature to leak into this
  # package, rcl/rmw would be missing and the loader would refuse to start it.
  # The guard is what a cross build needs — there the binary cannot run on the
  # machine that produced it, and the check is skipped rather than failed.
  doInstallCheck = pkgs.stdenv.buildPlatform.canExecute pkgs.stdenv.hostPlatform;
  installCheckPhase = ''
    runHook preInstallCheck

    $out/bin/clipper --version
    $out/bin/clipper --help
    $out/bin/clipper tail --interface mcap --help > /dev/null

    # `ros` is not a value this build rejects at runtime — it is a clap variant
    # the feature never compiled, so the parse fails. Asserting that parse error
    # rather than merely a non-zero exit is what makes the check bite: a build
    # that leaked the feature would accept the flag and start a recorder, and a
    # started recorder's own exit status — whatever the timeout below makes of
    # it — says nothing about which interfaces the binary offers.
    if refusal=$(timeout 60 $out/bin/clipper tail --interface ros 2>&1 </dev/null); then
      echo "clipper-ros-free accepted --interface ros: the ros feature leaked into this package" >&2
      exit 1
    fi
    if ! grep -qF "invalid value 'ros'" <<< "$refusal"; then
      echo "clipper-ros-free did not refuse --interface ros at parse time; it said:" >&2
      echo "$refusal" >&2
      exit 1
    fi

    runHook postInstallCheck
  '';

  meta = {
    description = "Triggered MCAP clip recorder, built without ROS 2";
    mainProgram = "clipper";
    license = pkgs.lib.licenses.asl20;
  };
}
