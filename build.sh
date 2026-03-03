#!/bin/bash
source /opt/ros/humble/setup.bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
