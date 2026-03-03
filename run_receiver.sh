#!/bin/bash
source /opt/ros/humble/setup.bash
LOG_LEVEL=1
./build/receiver_node config.json
