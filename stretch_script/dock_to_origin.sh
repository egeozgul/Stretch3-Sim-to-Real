#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

# ── Logging ────────────────────────────────────────────────────────────────
LOG_FILE=~/stretch_script/logs/dock_to_origin_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== dock_to_origin.sh started at $(date) ==="

# ── Cleanup on Ctrl+C or exit ──────────────────────────────────────────────
cleanup() {
  echo "=== Cleaning up... ==="
  kill $NAV_PID 2>/dev/null
  pkill -f "navigation.launch.py"
  stretch_free_robot_process.py || sudo -E env PATH=$PATH stretch_free_robot_process.py
  echo "=== Cleanup done ==="
  exit 0
}
trap cleanup SIGINT SIGTERM

# ── Free robot process ─────────────────────────────────────────────────────
echo "Freeing robot process..."
stretch_free_robot_process.py

# ── Release LiDAR port ─────────────────────────────────────────────────────
echo "Releasing LiDAR port..."
sudo fuser -k /dev/ttyUSB2 2>/dev/null
sleep 1

# ── Launch Nav2 ────────────────────────────────────────────────────────────
echo "Launching Nav2..."
ros2 launch stretch_nav2 navigation.launch.py \
  map:=${HELLO_FLEET_PATH}/maps/testing_map.yaml \
  teleop_type:=joystick \
  mode:=navigation &
NAV_PID=$!

# ── Wait for Nav2 ──────────────────────────────────────────────────────────
echo "Waiting for Nav2 action server..."
until ros2 action info /navigate_to_pose > /dev/null 2>&1; do
  sleep 1
done
echo "Nav2 is up!"

# ── Check LiDAR is alive ───────────────────────────────────────────────────
echo "Checking LiDAR..."
LIDAR_TIMEOUT=10
LIDAR_COUNT=0
until ros2 topic hz /scan --window 1 > /dev/null 2>&1 & sleep 2 && kill $! 2>/dev/null; do
  LIDAR_COUNT=$((LIDAR_COUNT + 1))
  if [ $LIDAR_COUNT -ge $LIDAR_TIMEOUT ]; then
    echo "ERROR: LiDAR not publishing. Check USB connection. Aborting."
    cleanup
  fi
  sleep 1
done
echo "LiDAR OK!"

# ── Wait for AMCL to be ready, then set initial pose ──────────────────────
echo "Waiting for AMCL service..."
until ros2 service list 2>/dev/null | grep -q "/amcl/get_state"; do
  sleep 1
done
sleep 2  # give AMCL a moment to fully activate

echo "Setting initial pose..."
until ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
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
}" && ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  echo "Waiting for AMCL to accept pose..."
  sleep 2
done

echo "AMCL localized!"

# ── Wait for controller_server ─────────────────────────────────────────────
echo "Waiting for controller_server..."
until ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance > /dev/null 2>&1; do
  sleep 1
done
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.035
echo "Yaw tolerance set."

# ── Switch to navigation mode ──────────────────────────────────────────────
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}
sleep 1

# ── Navigate to origin ─────────────────────────────────────────────────────
echo "=== Navigating to origin ==="
NAV_OUTPUT=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" 2>&1)
echo "$NAV_OUTPUT"

if echo "$NAV_OUTPUT" | grep -q "Goal finished with status: SUCCEEDED"; then
  echo "Navigation to origin succeeded!"
else
  echo "ERROR: Navigation did not succeed."
  cleanup
fi

echo "=== dock_to_origin.sh finished at $(date) ==="
cleanup
