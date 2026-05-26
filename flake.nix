{
  description = "Rust ROS2 subscriber (r2r) that attaches to every live topic";

  # nixpkgs has no ROS2; nix-ros-overlay packages the full Jazzy distro.
  # Track the same overlay as ../ros2_sources so both shells share one RMW.
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  # Pull prebuilt ROS2 packages from the ROS binary cache instead of compiling.
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

        # r2r generates Rust bindings (at `cargo build`) for every message
        # package on AMENT_PREFIX_PATH. List the packages whose types appear in
        # bags/example-011-ugv-ds.mcap plus the core rcl/rmw stack r2r links
        # against. The matching IDL_PACKAGE_FILTER below restricts codegen to
        # exactly these (r2r does no dependency resolution, so list them all).
        rosEnv = ros.buildEnv {
          paths = with ros; [
            # core client library + Fast DDS RMW that r2r builds against
            rcl
            rcl-action
            rmw-fastrtps-cpp
            # CLI, handy for `ros2 topic list` from the same shell
            ros2cli
            ros2cli-common-extensions
            # message packages carried by the UGV bag (see ../ros2_sources/REPLAY.md)
            builtin-interfaces
            std-msgs
            sensor-msgs
            geometry-msgs
            nav-msgs
            tf2-msgs
            velodyne-msgs
            rosgraph-msgs       # /clock when replaying with --clock
            # pulled in by rcl/actions; r2r's codegen needs them present
            action-msgs
            unique-identifier-msgs
            std-srvs
            # rclrs's vendored interfaces (rclrs/src/vendor) unconditionally link
            # their typesupport C libs, so these must be on the link path even
            # though no bag topic uses them — see ros2-rust/ros2_rust#557. r2r
            # ignores them.
            example-interfaces
            test-msgs
          ];
        };
      in {
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

          # Restrict r2r codegen to just the bag's packages — semicolon
          # separated, no auto dependency resolution, so every used package is
          # listed explicitly. Keeps the first build fast.
          IDL_PACKAGE_FILTER =
            "builtin_interfaces;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;tf2_msgs;velodyne_msgs;rosgraph_msgs;action_msgs;unique_identifier_msgs;std_srvs";

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
