if [ "$ROS2_ROOT" == "" ]; then
  echo "ROS2_ROOT not set. Please set this environment variable to your system's ROS2 workspace."
  exit 1
fi

cwd=$(dirname $BASH_SOURCE)

source $ROS2_ROOT/install/setup.bash
source $cwd/../ros2_ws/install/local_setup.bash
