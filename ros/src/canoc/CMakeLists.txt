cmake_minimum_required(VERSION 2.8)
project(canoc)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs opencaret_msgs rospy)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs opencaret_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
