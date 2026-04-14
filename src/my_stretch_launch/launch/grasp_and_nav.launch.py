#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

echo "=== PHASE 1: Navigation ==="

# Launch Nav2
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/testing_map.yaml \
  teleop_type:=joystick \
  mode:=navigation &
NAV_PID=$!

echo "Waiting for Nav2 to start..."
sleep 15

# Set initial pose and switch to navigation mode
~/stretch_script/set_initial_pose.sh
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.035

echo "Docking to table..."
~/stretch_script/dock_to_table.sh

echo "=== PHASE 2: Grasp pipeline starting ==="

# 1. Stretch driver
gnome-terminal --title="stretch_driver" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 launch stretch_core stretch_driver.launch.py
" &
sleep 3

# 2. D435i camera
gnome-terminal --title="d435i_camera" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 launch stretch_core d435i_high_resolution.launch.py
" &
sleep 3

# 3. ArUco detector
gnome-terminal --title="aruco_detector" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  python3 ~/tf.py
" &
sleep 5

# 4. Grasper
gnome-terminal --title="object_grasper" -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ament_ws/install/setup.bash
  ros2 run stretch_perception altobject
" &
sleep 3

echo ""
echo "=== READY ==="
echo "All grasp nodes up. Triggering grasp in 10 seconds..."
sleep 10

ros2 service call /grasp_object std_srvs/srv/Trigger {}
echo "Grasp triggered!"
