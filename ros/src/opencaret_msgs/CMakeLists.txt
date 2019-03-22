cmake_minimum_required(VERSION 2.8)
project(opencaret_msgs)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
	FILES Time.msg LongitudinalTarget.msg CanMessage.msg RadarTrack.msg RadarTrackAccel.msg RadarTracks.msg LongitudinalPlan.msg Obstacle.msg Obstacles.msg RoadSurface.msg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
