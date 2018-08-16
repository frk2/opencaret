cwd=$(dirname "$0")
cmd=$(which ros2)
if [ "$cmd" == "" ]; then
  echo "ros2 unavailable... did you 'source ./source_ros2.sh' ?"
  exit 1
fi

pushd $cwd/../ros2_ws/
ros2 launch src/launch/all_launch.py
