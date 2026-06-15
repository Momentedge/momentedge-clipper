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
    extra-substituters = ["https://ros.cachix.org"];
    extra-trusted-public-keys = ["ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="];
  };

  outputs = {
    nixpkgs,
    nix-ros-overlay,
    ...
  }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [nix-ros-overlay.overlays.default];
      };
      lib = pkgs.lib;

      # The ROS2 distros this repo is built and tested against. nix-ros-overlay
      # packages each one as `pkgs.rosPackages.<distro>`; everything below
      # (dev shell, nix-built binaries, the recorder closure) is produced once
      # per distro by `mkDistro`. Pick a distro at the command line:
      #   nix develop .#humble        nix build .#edgestream-rec-cont-rolling
      # The overlay also ships `kilted`; add it here to build against it.
      rosDistros = ["humble" "jazzy" "lyrical" "rolling"];

      # The distro used when no selector is given — `nix develop` and the
      # unsuffixed packages (`nix build .#edgestream-rec-cont`). Jazzy is the LTS the
      # bench is tuned on. The deployment target builds natively against its own
      # apt ROS2 (Humble; see README "Native build on the target"), independent
      # of this and of the nix-built outputs.
      defaultDistro = "jazzy";

      # Distros whose dev shell carries the synthetic gscam camera (sim/). The
      # overlay packages the sim stack (gscam, ffmpeg_image_transport) only for
      # Jazzy; on Humble/Lyrical/Rolling those derivations fail to configure, so
      # their shells ship the recorder/e2e core without it (nix/ros-env.nix
      # `withSim`). The recorder, the e2e suite, and deployment use none of the
      # sim packages, so this gates off cleanly. Extend this list as the overlay
      # gains sim support for more distros.
      simDistros = [ "jazzy" ];

      # r2r does no dependency resolution, so codegen must be handed every
      # used message package explicitly. This filter covers the two packages
      # the kept crates decode: edgestream_msgs (Trigger/Recorded) and its
      # builtin_interfaces dependency (Time). It drives both the dev shell
      # (system cargo) and the nix-built binaries, for every distro. The
      # packages listed exist under all targeted distros, so one filter serves
      # them all.
      idlPackageFilter = "builtin_interfaces;edgestream_msgs";

      # GStreamer plugin set the sim camera's gscam pipeline draws from
      # (sim/cam_sim.sh). gscam's own closure carries only core +
      # plugins-base (enough for videotestsrc and videoconvert); the rest
      # cover encoders/parsers for manual gst-launch experimentation and any
      # future pipeline element. Distro-independent, so built once.
      gstPlugins = with pkgs.gst_all_1; [
        gstreamer
        gst-plugins-base
        gst-plugins-good
        gst-plugins-bad
        gst-plugins-ugly
        gst-libav
      ];

      # Everything anchored to one ROS2 distro. Package definitions live under
      # ./nix; source paths are passed in from here so they stay anchored at the
      # repo root. Flakes only see git-tracked files — a newly added file under
      # nix/ or edgestream_msgs/ must be `git add`ed before the eval sees it.
      mkDistro = rosDistro: let
        ros = pkgs.rosPackages.${rosDistro};
        withSim = lib.elem rosDistro simDistros;
        edgestream-msgs = import ./nix/edgestream-msgs.nix {
          inherit ros;
          src = ./edgestream_msgs;
        };
        rosEnv = import ./nix/ros-env.nix {inherit ros edgestream-msgs lib withSim;};
        binaries = import ./nix/binaries.nix {
          inherit pkgs rosEnv idlPackageFilter rosDistro;
          src = ./.;
          cargoLockFile = ./Cargo.lock;
        };

        devShell = pkgs.mkShell {
          name = "ros2-rust-subscribe-${rosDistro}";
          # Rust itself is intentionally NOT provided here — use the system
          # cargo/rustc (whatever is on PATH). The shell only supplies the ROS2
          # stack and the C toolchain r2r's build script needs.
          packages =
            [
              rosEnv
              pkgs.clang # r2r's build script invokes clang/bindgen
              pkgs.pkg-config
            ]
            # the sim camera's GStreamer pipeline (sim/) — only where the sim
            # stack is in the closure (see simDistros).
            ++ lib.optionals withSim gstPlugins;

          # bindgen (via r2r_common) needs to find libclang.
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";

          # Restrict r2r codegen to just the bag's packages (shared with the
          # nix-built binaries below). Keeps the first build fast.
          IDL_PACKAGE_FILTER = idlPackageFilter;

          # Match ../ros2_sources so discovery + SHM line up: same RMW, same
          # domain. ROS_DISTRO is the standard ROS variable exported alongside
          # them; r2r does not need it but is unaffected.
          shellHook = ''
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export ROS_DOMAIN_ID=''${ROS_DOMAIN_ID:-0}
            export ROS_DISTRO=${rosDistro}
            ${lib.optionalString withSim ''
              # Let the GStreamer pipeline gscam spawns (sim/) find the plugins.
              export GST_PLUGIN_SYSTEM_PATH_1_0="${lib.makeSearchPathOutput "lib" "lib/gstreamer-1.0" gstPlugins}"
            ''}
            echo "ROS2 ${rosDistro} rust-subscribe shell — RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID  DISTRO=$ROS_DISTRO"
          '';
        };
      in {
        inherit rosEnv binaries devShell;
      };

      # One built attrset per distro, lazily — `nix develop .#humble` forces
      # only humble's devShell, never the others.
      distros = lib.genAttrs rosDistros mkDistro;

      # Per-distro package outputs: rosEnv-<distro>,
      # edgestream-rec-cont-<distro>, trigger-pub-<distro>.
      perDistroPackages =
        lib.concatMapAttrs (distro: d: {
          "rosEnv-${distro}" = d.rosEnv;
          "edgestream-rec-cont-${distro}" = d.binaries.edgestream-rec-cont;
          "trigger-pub-${distro}" = d.binaries.trigger-pub;
        })
        distros;

      # Unsuffixed aliases for the default distro, so
      # `nix build .#edgestream-rec-cont` keeps working.
      defaultPackages = {
        rosEnv = distros.${defaultDistro}.rosEnv;
        inherit
          (distros.${defaultDistro}.binaries)
          edgestream-rec-cont
          trigger-pub
          ;
      };
    in {
      # rosEnv (the dev shell's ROS2 closure) and the nix-built binaries are
      # exposed mostly as build checks — `nix build .#edgestream-rec-cont-rolling`
      # compiles the deployable under nix, against that distro, without the
      # system cargo. The target deploys native apt builds, not these (see
      # README "Deployment").
      packages = perDistroPackages // defaultPackages;

      # `nix develop .#<distro>` selects a distro; bare `nix develop` is the
      # default (jazzy). Each shell pins ROS_DISTRO and the matching ROS2 closure.
      devShells =
        lib.mapAttrs (_distro: d: d.devShell) distros
        // {
          default = distros.${defaultDistro}.devShell;
        };
    });
}
