#!/bin/bash

cwd=$(dirname "$BASH_SOURCE")

echo $@

USE_ZED=1
DEVICE_PATH=/dev/video1

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

echo "Launching zed_wrapper_video_record_left.launch"
LAUNCH_FILE=$cwd/../ros1_ws/src/launch/visualize.launch

roslaunch $LAUNCH_FILE
