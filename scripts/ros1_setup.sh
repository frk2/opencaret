trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

cwd=$(dirname "$BASH_SOURCE")
cmd=$(which roscore)
if [ "$cmd" == "" ]; then
  echo "ros1 unavailable... did you 'source ./source_ros1.sh' ?"
  exit 1
fi

cmd="$(rostopic list 2>&1)"

if [[ "$cmd" =~ "ERROR" ]]
then
  echo 'Launching roscore'
  roscore &
  sleep 5
fi

echo 'Launching ros1_bridge'

$cwd/ros1_bridge.sh
