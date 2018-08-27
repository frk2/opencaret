cwd=$(dirname "$0")
cmd=$(which ros2)
if [ "$cmd" == "" ]; then
  echo "ros2 unavailable... did you 'source ./source_ros2.sh' ?"
  exit 1
fi

MODULES=1
POSITIONAL=()

execute () {
  if [ "$MODULES" == "1" ]; then
    $1 ${POSITIONAL[@]}
  else
    $1 ${POSITIONAL[@]} &
    sleep 2
  fi
  MODULES=$((MODULES - 1))
}

trap "trap - SIGTERM && sleep 1 && kill -- -$$" SIGINT SIGTERM EXIT

ENABLE_BRIDGE=0
LAUNCH_FILE="$cwd/../ros2_ws/src/launch/all_launch.py"

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --bridge)
    ENABLE_BRIDGE=1
    MODULES=$((MODULES + 1))
    shift
    ;;
    --sensors)
    LAUNCH_FILE="$cwd/../ros2_ws/src/launch/sensors_launch.py"
    shift
    ;;
    -h|--help)
    shift
    echo "launch_ros2.sh [--bridge] [--sensors]"
    exit 0
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

if [ "$ENABLE_BRIDGE" == "1" ]; then
  export ROS_MASTER_URI=http://localhost:11311
  echo 'Launching ROS1 Bridge'
  execute "ros2 run ros1_bridge dynamic_bridge"
fi

execute "ros2 launch $LAUNCH_FILE"
