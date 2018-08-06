USE_ZED_NODE=1

if [ "$USE_ZED_NODE" == "1" ]; then
  VIDEO_TOPIC1="/zed/rgb/image_rect_color/stream"
  VIDEO_TOPIC2="/zed/rgb/image_rect_color/stream/event"
else
  VIDEO_TOPIC1="/zed/rgb/image_raw_color/stream"
  VIDEO_TOPIC2="/zed/rgb/image_raw_color/stream/event"
fi

rosbag record \
/can_recv \
/can_send \
/radar_tracks \
/robot_description \
/rosout \
/rosout_agg \
$VIDEO_TOPIC1 \
$VIDEO_TOPIC2
