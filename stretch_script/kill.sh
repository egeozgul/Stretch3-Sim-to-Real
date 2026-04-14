#!/bin/bash
echo "Killing all ROS2 and robot processes..."
pkill -9 -f ros2
pkill -9 -f python3
pkill -9 -f stretch

pkill -f "realsense2_camera_node"
pkill -f "d435i"
pkill -f "d405"

sleep 2
sudo -E env PATH=$PATH stretch_free_robot_process.py
echo "Done! Wait 3 seconds before launching again..."
sleep 3
