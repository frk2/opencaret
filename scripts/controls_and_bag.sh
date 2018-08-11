
rostopic pub /controls_enable std_msgs/Bool "data: True" -1 &
rosbag record \
/can_recv \
/can_send \
/radar_tracks \
/robot_description \
/rosout \
/rosout_agg \
/radar_viz \
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
/wheel_speed
#/zed/joint_states \
#/zed/left/image_rect_color \
#/zed/left/image_rect_color/camera_info \
#/zed/odom \
#/zed/point_cloud/cloud_registered \
#/zed/depth/camera_info \
#/zed/depth/depth_registered \
