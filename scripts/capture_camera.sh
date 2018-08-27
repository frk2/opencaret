#!/bin/bash

cwd=$(dirname "$BASH_SOURCE")

echo $@

USE_ZED=1
ENABLE_RECORD=1
ENABLE_CAPTURE=1
DEVICE_PATH=/dev/video1

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --zed)
    USE_ZED=1
    shift # past argument
    ;;
    --libav)
    USE_ZED=0
    shift # past argument
    ;;
    --no-capture)
    ENABLE_CAPTURE=0
    shift # past argument
    ;;
    --no-record)
    ENABLE_RECORD=0
    shift # past argument
    ;;
    -d|--device)
    DEVICE_PATH=$2
    shift
    shift # past argument
    ;;
    -h|--help)
    shift
    echo "capture_camera.sh [--zed|--libav] [-d|--device device_path] [-h|--help]"
    exit 0
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

if [ "$USE_ZED" == "1" ]; then
  echo "Launching zed_wrapper_video_record_left.launch"
  LAUNCH_FILE=$cwd/../ros1_ws/src/launch/zed_wrapper_video_record_left.launch
else
  echo "Launching zed_libav_video_record_left.launch"
  LAUNCH_FILE=$cwd/../ros1_ws/src/launch/zed_libav_video_record_left.launch
fi

roslaunch $LAUNCH_FILE device_path:=$DEVICE_PATH capture:=$ENABLE_CAPTURE record:=$ENABLE_RECORD
