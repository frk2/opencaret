source $ROS1_ROOT/setup.bash
. $ROS2_ROOT/install/local_setup.bash
source ../ros1_ws/devel/setup.bash
. ../ros2_ws/install/local_setup.bash
cd $ROS2_ROOT
rm -rf build/ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
