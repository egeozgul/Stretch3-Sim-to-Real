#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

LOG_FILE=~/stretch_script/logs/return_home_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== return_home.sh started at $(date) ==="

READY_FLAG=/tmp/stretch_nodes_ready

# ── Wait for launch_nodes.sh to signal readiness ───────────────────────────
echo "Waiting for nodes to be ready (launch_nodes.sh)..."
READY_TIMEOUT=120; READY_COUNT=0
until [ -f "$READY_FLAG" ]; do
  sleep 1; READY_COUNT=$((READY_COUNT + 1))
  if [ $READY_COUNT -ge $READY_TIMEOUT ]; then
    echo "ERROR: Nodes not ready after ${READY_TIMEOUT}s. Did you run launch_nodes.sh? Aborting."
    exit 1
  fi
done
echo "Nodes are ready!"

# ── Switch to navigation mode and tune yaw tolerance ──────────────────────
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}

echo "Waiting for controller_server..."
until ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance > /dev/null 2>&1; do
  sleep 1
done
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.035
echo "Yaw tolerance set."

# ── Navigate back to origin ────────────────────────────────────────────────
echo "=== Navigating back to origin ==="
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
  echo "ERROR: Navigation to origin did not succeed. Aborting."
  exit 1
fi

# ── Reset arm to default position ─────────────────────────────────────────
echo "Resetting arm position..."
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger {}
sleep 1

python3 ~/test1.py; EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 139 ]; then
  echo "ERROR: test1.py failed (exit $EXIT_CODE). Aborting."
  exit 1
fi
echo "Arm reset complete (exit $EXIT_CODE)."

echo "=== return_home.sh finished at $(date) ==="
