VIDEO_TOPIC1="/zed/left/image_raw_color/stream"
VIDEO_TOPIC2="/zed/left/image_raw_color/stream/event"

rosbag record \
/can_recv \
/can_send \
/radar_tracks \
/robot_description \
/rosout \
/rosout_agg \
$VIDEO_TOPIC1 \
$VIDEO_TOPIC2
