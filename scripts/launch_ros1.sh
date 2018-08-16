#!/bin/bash

MODULES=1
POSITIONAL=()

execute () {
  echo "Modules $MODULES"
  if [ "$MODULES" == "1" ]; then
    $1 ${POSITIONAL[@]}
  else
    $1 ${POSITIONAL[@]} &
    sleep 2
  fi
  MODULES=$((MODULES - 1))
}

trap "trap - SIGTERM && sleep 1 && kill -- -$$" SIGINT SIGTERM EXIT

cwd=$(dirname "$BASH_SOURCE")

ENABLE_VIZ=0
ENABLE_CAMERA=0
ENABLE_RECORD=1
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --viz)
    ENABLE_VIZ=1
    MODULES=$((MODULES + 1))
    shift
    ;;
    --no-record)
    ENABLE_RECORD=0
    MODULES=$((MODULES - 1))
    shift
    ;;
    --camera)
    ENABLE_CAMERA=1
    MODULES=$((MODULES + 1))
    shift
    ;;
    -h|--help)
    shift
    echo "run_ros1.sh [--viz] [--record] [--camera]"
    exit 0
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

if [ "$ENABLE_RECORD" == "1" ]; then
  execute "$cwd/record_bag.sh"
fi

if [ "$ENABLE_CAMERA" == "1" ]; then
  execute "$cwd/capture_camera.sh"
fi

if [ "$ENABLE_VIZ" == "1" ]; then
  execute "rosrun radar radar_viz.py"
fi
