#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

echo "=== Returning to origin ==="

# Switch to navigation mode
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}

# Set current known pose (docking position) so AMCL knows where we are
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.5688, y: 1.6601, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
    },
    covariance: [0.25, 0, 0, 0, 0, 0,
                 0, 0.25, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0.07]
  }
}"

sleep 2

# Tighten yaw tolerance
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.035

# Navigate to origin (0, 0, 0°)
echo "Navigating to origin..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"

echo "=== Arrived at origin ==="
