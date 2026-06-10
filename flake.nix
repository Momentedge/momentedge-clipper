{
  description = "Rust ROS2 recorders + synthetic gscam camera source (raw + H.265)";

  # nixpkgs has no ROS2; nix-ros-overlay packages the full ROS2 distro.
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
        # The single source of truth for the ROS2 distro the nix dev shell and
        # nix-built binaries target. Drives the package set below and the
        # ROS_DISTRO env (r2r's codegen is distro-agnostic, but rclrs's build.rs
        # selects its committed rcl bindings by it). Switching distros is this
        # line. The deployment target builds natively against its own apt ROS2
        # (see README "Native build on the target"), independent of this.
        rosDistro = "jazzy";
        ros = pkgs.rosPackages.${rosDistro};

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
          inherit pkgs rosEnv idlPackageFilter rosDistro;
          src = ./.;
          cargoLockFile = ./Cargo.lock;
        };

        # GStreamer plugin set the sim camera's gscam pipeline draws from
        # (sim/cam_sim.sh). gscam's own closure carries only core +
        # plugins-base (enough for videotestsrc and videoconvert); the rest
        # cover encoders/parsers for manual gst-launch experimentation and any
        # future pipeline element.
        gstPlugins = with pkgs.gst_all_1; [
          gstreamer
          gst-plugins-base
          gst-plugins-good
          gst-plugins-bad
          gst-plugins-ugly
          gst-libav
        ];
      in {
        # rosEnv (the dev shell's ROS2 closure) and the two nix-built binaries
        # are exposed mostly as build checks — `nix build .#edgestream-rec`
        # compiles the deployable under nix without the system cargo. The target
        # deploys native apt builds, not these (see README "Deployment").
        packages = {
          inherit rosEnv;
          inherit (binaries) edgestream-rec edgestream-rec-cont trigger-pub;
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
          ] ++ gstPlugins;    # the sim camera's GStreamer pipeline (sim/)

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
            export ROS_DISTRO=${rosDistro}
            # Let the GStreamer pipeline gscam spawns (sim/) find the plugins.
            export GST_PLUGIN_SYSTEM_PATH_1_0="${pkgs.lib.makeSearchPathOutput "lib" "lib/gstreamer-1.0" gstPlugins}"
            echo "ROS2 ${rosDistro} rust-subscribe shell — RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID  DISTRO=$ROS_DISTRO"
          '';
        };
      });
}
