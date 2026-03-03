#!/bin/bash
source /opt/ros/humble/setup.bash
LOG_LEVEL=1
./build/sender_node config.json
