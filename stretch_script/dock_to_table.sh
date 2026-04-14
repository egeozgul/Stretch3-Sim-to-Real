#!/bin/bash
source ~/ament_ws/install/setup.bash
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}

sleep 2

ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.035

sleep 1

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.5688, y: 1.6601, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
    }
  }
}"
