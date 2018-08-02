source ./source.sh
cd $ROS2_ROOT
rm -rf build/ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
