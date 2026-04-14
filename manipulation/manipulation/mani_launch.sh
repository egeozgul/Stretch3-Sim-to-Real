#!/bin/bash
# Launch all Stretch grasp pipeline components in separate terminals
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

echo "Starting Stretch grasp pipeline..."

# 1. Stretch driver
gnome-terminal --title="stretch_driver" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 launch stretch_core stretch_driver.launch.py
" &
sleep 3  # wait for driver to come up

# 2. D435i camera
gnome-terminal --title="d435i_camera" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 launch stretch_core d435i_high_resolution.launch.py
" &
sleep 3  # wait for camera

# 3. ArUco detector (marker_pose.py)
gnome-terminal --title="detector" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  python3 ~/manipulation/manipulation/marker_pose.py
" &
sleep 5  # wait for aruco to initialize and move head

# 4. Object grasper node
gnome-terminal --title="object_grasper" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 run manipulation grasper
" &
sleep 3

echo ""
echo "All nodes launched!"
echo ""
echo "When ready to grasp, run:"
echo "  ros2 service call /grasp_object std_srvs/srv/Trigger {}"
