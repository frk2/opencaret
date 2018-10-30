
cmake_minimum_required(VERSION 3.5)

project(opencaret_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(opencaret_msgs
  "msg/Time.msg"
  "msg/CanMessage.msg"
  "msg/RadarTrack.msg"
  "msg/RadarTrackAccel.msg"
  "msg/RadarTracks.msg"
  "msg/LongitudinalPlan.msg"
  "msg/Obstacle.msg"
  "msg/Obstacles.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
