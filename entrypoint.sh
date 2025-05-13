#!/bin/bash

WS_DIR=~/esp_daemon

cd $WS_DIR
# . /opt/ros/humble/setup.sh && colcon build --symlink-install

source $WS_DIR/install/setup.bash
sudo chmod 666 /dev/ttyACM0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# ./micro-ROS_install.sh

while true; do
    sleep 60
done

exec "$@"