# Local ROS2 interface package defining the edgestream Trigger/Recorded event
# messages. Built with the standard rosidl generators, exactly like any other
# ament_cmake message package, so its typesupport lands on AMENT_PREFIX_PATH for
# both r2r codegen and the `ros2` CLI. Mirrors the upstream example-interfaces
# expression. `src` is passed in by the flake so the path stays anchored at the
# repo root rather than this file's directory.
{ ros, src }:

ros.buildRosPackage {
  pname = "edgestream_msgs";
  version = "0.0.1";
  inherit src;
  buildType = "ament_cmake";
  buildInputs = with ros; [ ament-cmake rosidl-default-generators builtin-interfaces ];
  propagatedBuildInputs = with ros; [ rosidl-default-runtime builtin-interfaces ];
  nativeBuildInputs = with ros; [ ament-cmake rosidl-default-generators ];
  meta.description = "edgestream Trigger/Recorded event interfaces";
}
