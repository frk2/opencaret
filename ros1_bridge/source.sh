if [ "$ROS1_ROOT" == "" ]; then
  echo "ROS1_ROOT not set. Please set this environment variable to your system's ROS1 path."
  exit 1
fi

if [ "$ROS2_ROOT" == "" ]; then
  echo "ROS2_ROOT not set. Please set this environment variable to your system's ROS2 workspace."
  exit 1
fi

source $ROS1_ROOT/setup.bash
source $ROS2_ROOT/install/setup.bash
source ../ros1_ws/devel/setup.bash
source ../ros2_ws/install/local_setup.bash
