function finish {

  popd
  #re-enable zed 
  mv ../ros1_ws/src/.zed-ros-wrapper ../ros1_ws/src/zed-ros-wrapper
}

trap finish EXIT

source ./source.sh

# hide zed_wrapper (it is not yet compatible with melodic since it uses a missing opencv3 package)
mv ../ros1_ws/src/zed-ros-wrapper ../ros1_ws/src/.zed-ros-wrapper

pushd $ROS2_ROOT
rm -rf build/ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
