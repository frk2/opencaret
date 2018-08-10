#!/bin/bash
rm -rf ./build/ ./install
colcon build --symlink-install
