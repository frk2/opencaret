cmake_minimum_required(VERSION 2.8)
project(planner)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
