if [ "$ROS1_ROOT" == "" ]; then
  echo "ROS1_ROOT not set. Please set this environment variable to your system's ROS1 path."
  exit 1
fi

cwd=$(dirname $BASH_SOURCE)

source $ROS1_ROOT/setup.bash
source $cwd/../ros/devel/setup.bash
