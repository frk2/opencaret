trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

cwd=$(dirname "$BASH_SOURCE")
cmd=$(which roscore)
if [ "$cmd" == "" ]; then
  echo "ros1 unavailable... did you 'source ./source_ros1.sh' ?"
  exit 1
fi

# launch playback

roslaunch $cwd/../ros/src/launch/visualize.launch playback:=1 &

# rosbag play
$cwd/play_bag.sh $@
