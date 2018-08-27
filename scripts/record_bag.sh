cwd=$(dirname "$BASH_SOURCE")

function terminate {
  echo 'Terminating recording...'
  sleep 5
  trap - SIGTERM
}

# sleep here to continue recording while other nodes continue to shutdown
trap terminate SIGINT SIGTERM

RECORD_CAMERA=1
RECORD_TF=0  # should these ever be recorded? These can be published by a node during playback
RECORD_LONGITUDINAL=1
RECORD_SENSOR=1

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --record-tf)
    RECORD_TF=1
    shift
    ;;
    --no-record-camera)
    RECORD_CAMERA=0
    shift
    ;;
    --no-record-longitudinal)
    RECORD_LONGITUDINAL=0
    shift
    ;;
    --no-record-sensor)
    RECORD_SENSOR=0
    shift
    ;;
    -h|--help)
    shift
    echo "record_bag.sh [--record-tf] [--no-record-longitudinal]"
    exit 0
    ;;
esac
done

if [ "$RECORD_CAMERA" == "1" ]; then
  VIDEO_TOPIC="/zed/left/image_raw_color/stream \
  /zed/left/image_raw_color/stream/event \
  "
else
  VIDEO_TOPIC=""
fi

if [ "$RECORD_TF" == "1" ]; then
  TF_TOPICS="/tf \
  /tf_static \
  "
else
  TF_TOPICS=""
fi

if [ "$RECORD_LONGITUDINAL" == "1" ]; then
  LONG_TOPICS="
  /computed_accel_filtered \
  /computed_accel_raw \
  /controls_enable \
  /cruising_speed \
  /debug_target_speed \
  /imu \
  /lead_vehicle \
  /longitudinal_plan \
  /pid_ff \
  /pid_i \
  /pid_p \
  /pid_target_accel \
  /pid_target_speed \
  /plan_deviation \
  "
else
  LONG_TOPICS=""
fi

if [ "$RECORD_SENSOR" == "1" ]; then
  SENSOR_TOPICS="
  /radar_tracks \
  /steering/wheel_angle/raw \
  /steering/yaw_angle/raw \
  /steering/joint_states \
  /steering_torque \
  /throttle_command \
  /wheel_speed \
  "
else
  SENSOR_TOPICS=""
fi

pushd $cwd/../data

rosbag record \
/can_recv \
/can_send \
/robot_description \
/rosout \
/rosout_agg \
$SENSOR_TOPICS \
$TF_TOPICS \
$LONG_TOPICS \
$VIDEO_TOPIC \
#/zed/joint_states \
#/zed/left/image_rect_color \
#/zed/left/image_rect_color/camera_info \
#/zed/odom \
#/zed/point_cloud/cloud_registered \
#/zed/depth/camera_info \
#/zed/depth/depth_registered \

echo 'Terminated recording.'

popd
