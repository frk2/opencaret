cwd=$(dirname "$BASH_SOURCE")

function terminate {
  echo 'Terminating recording...'
  sleep 5
  trap - SIGTERM
}

# sleep here to continue recording while other nodes continue to shutdown
trap terminate SIGINT SIGTERM

USE_CAMERA=1

if [ "$USE_CAMERA" == "1" ]; then
  VIDEO_TOPIC1="/zed/left/image_raw_color/stream"
  VIDEO_TOPIC2="/zed/left/image_raw_color/stream/event"
else
  VIDEO_TOPIC1=""
  VIDEO_TOPIC2=""
fi

pushd $cwd/../data

rosbag record \
/can_recv \
/can_send \
/radar_tracks \
/robot_description \
/rosout \
/rosout_agg \
/tf \
/tf_static \
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
/steering_angle \
/steering_torque \
/throttle_command \
/wheel_speed \
$VIDEO_TOPIC1 \
$VIDEO_TOPIC2
#/zed/joint_states \
#/zed/left/image_rect_color \
#/zed/left/image_rect_color/camera_info \
#/zed/odom \
#/zed/point_cloud/cloud_registered \
#/zed/depth/camera_info \
#/zed/depth/depth_registered \

echo 'Terminated recording.'

popd
