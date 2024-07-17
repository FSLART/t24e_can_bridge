#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run t24e_can_bridge bridge &
ros2 launch spac2_0 drivemodel.launch.xml
