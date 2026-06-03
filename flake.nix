{
  description = "Rust ROS2 subscriber (r2r) that attaches to every live topic";

  # nixpkgs has no ROS2; nix-ros-overlay packages the full Jazzy distro.
  # Track the same overlay as ../ros2_sources so both shells share one RMW.
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  # Pull prebuilt ROS2 packages from the ROS binary cache instead of compiling.
  # The aarch64 deployment closure additionally needs the @wentasah attic cache
  # (https://attic.iid.ciirc.cvut.cz/ros) — see nix/README.md.
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  outputs = { nixpkgs, nix-ros-overlay, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        ros = pkgs.rosPackages.jazzy;

        # r2r does no dependency resolution, so codegen must be handed every
        # used message package explicitly. This single filter drives both the
        # dev shell (system cargo) and the nix-built binaries — keep it and
        # nix/ros-env.nix's package list in step.
        idlPackageFilter =
          "builtin_interfaces;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;tf2_msgs;velodyne_msgs;rosgraph_msgs;action_msgs;unique_identifier_msgs;std_srvs;rosbag2_interfaces;edgestream_msgs";

        # Package definitions live under ./nix; the source paths are passed in
        # from here so they stay anchored at the repo root. Flakes only see
        # git-tracked files — a newly added file under nix/ or edgestream_msgs/
        # must be `git add`ed before the eval sees it.
        edgestream-msgs = import ./nix/edgestream-msgs.nix {
          inherit ros;
          src = ./edgestream_msgs;
        };
        rosEnv = import ./nix/ros-env.nix { inherit ros edgestream-msgs; };
        binaries = import ./nix/binaries.nix {
          inherit pkgs rosEnv idlPackageFilter;
          src = ./.;
          cargoLockFile = ./Cargo.lock;
        };
        images = import ./nix/images.nix { inherit pkgs rosEnv binaries; };
      in {
        # rosEnv is exposed for the aarch64 deployment probe: dry-run whether the
        # full ROS2 closure is substitutable for a given system without building
        # anything (`nix build --dry-run .#packages.aarch64-linux.rosEnv`).
        packages = {
          inherit rosEnv;
          inherit (binaries) edgestream-rec trigger-pub;
          inherit (images) edgestream-rec-image trigger-pub-image;
        };

        devShells.default = pkgs.mkShell {
          name = "ros2-rust-subscribe";
          # Rust itself is intentionally NOT provided here — use the system
          # cargo/rustc (whatever is on PATH). The shell only supplies the ROS2
          # stack and the C toolchain r2r's build script needs.
          packages = [
            rosEnv
            pkgs.clang        # r2r's build script invokes clang/bindgen
            pkgs.pkg-config
          ];

          # bindgen (via r2r_common) needs to find libclang.
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";

          # Restrict r2r codegen to just the bag's packages (shared with the
          # nix-built binaries above). Keeps the first build fast.
          IDL_PACKAGE_FILTER = idlPackageFilter;

          # Match ../ros2_sources so discovery + SHM line up: same RMW, same domain.
          # ROS_DISTRO is required by rclrs's build.rs (it selects the committed
          # rcl bindings, e.g. rcl_bindings_generated_jazzy.rs, via a cfg flag and
          # then aborts if unset); r2r does not need it but is unaffected.
          shellHook = ''
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export ROS_DOMAIN_ID=''${ROS_DOMAIN_ID:-0}
            export ROS_DISTRO=jazzy
            echo "ROS2 Jazzy rust-subscribe shell — RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID  DISTRO=$ROS_DISTRO"
          '';
        };
      });
}
