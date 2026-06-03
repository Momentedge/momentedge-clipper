# OCI image per deployable, sharing the ROS2 closure as base layers. At runtime
# r2r dlopens the RMW via AMENT_PREFIX_PATH + RMW_IMPLEMENTATION, so both must be
# set in the image env. Produce the aarch64 variant with
# `.#packages.aarch64-linux.<name>-image`.
{ pkgs, rosEnv, binaries }:

let
  mkImage = pname: bin: pkgs.dockerTools.buildLayeredImage {
    # Image is named after the crate (edgestream-rec / trigger-pub); the
    # edgestream-rec image carries both the binary and `ros2 bag record`, so it
    # backs two containers (continuous recorder + extractor) — see scripts/.
    name = pname;
    tag = "jazzy";
    contents = [ bin rosEnv pkgs.bashInteractive pkgs.coreutils ];
    config = {
      Cmd = [ "${bin}/bin/${pname}" ];
      Env = [
        "AMENT_PREFIX_PATH=${rosEnv}"
        "LD_LIBRARY_PATH=${rosEnv}/lib"
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        "ROS_DOMAIN_ID=0"
        "ROS_DISTRO=jazzy"
        "PATH=/bin:${rosEnv}/bin:${bin}/bin"
      ];
    };
  };
in {
  edgestream-rec-image = mkImage "edgestream-rec" binaries.edgestream-rec;
  trigger-pub-image = mkImage "trigger-pub" binaries.trigger-pub;
}
