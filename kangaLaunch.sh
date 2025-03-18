#!/bin/bash

# Ensure Bluetooth is up
while ! systemctl is-active --quiet bluetooth.service; do
    echo "Waiting for Bluetooth..."
    sleep 2
done
echo "Bluetooth is active!"

# Bring up CAN interface
sudo ip link set can1 up type can bitrate 250000

# Wait for CAN interface to stabilize
sleep 2

# Launch ROS 2
source /home/tronsoc/kanga/kanga_launch/install/setup.bash 
source /home/tronsoc/kanga/telecoms/install/setup.bash
source /home/tronsoc/kanga/cameras/install/setup.bash
ros2 launch kanga_launch kanga_launch.py
