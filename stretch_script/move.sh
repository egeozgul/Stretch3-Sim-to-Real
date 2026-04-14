#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash

LOG_FILE=~/stretch_script/logs/dock_pos_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== dock_pos.sh started at $(date) ==="

# ── Named poses ────────────────────────────────────────────────────────────
POSES_FILE=~/stretch_script/poses.conf
declare -A POSES
while IFS='=' read -r name coords || [ -n "$name" ]; do
  [[ "$name" =~ ^#.*$ || -z "$name" ]] && continue  # skip comments/blank lines
  POSES["$name"]="$coords"
done < "$POSES_FILE"

# ── Parse arguments ────────────────────────────────────────────────────────
if [ $# -eq 1 ]; then
  POSE_NAME=$1
  if [ -z "${POSES[$POSE_NAME]}" ]; then
    echo "ERROR: Unknown pose '$POSE_NAME'. Available poses:"
    for name in "${!POSES[@]}"; do
      echo "  $name -> ${POSES[$name]}"
    done
    exit 1
  fi
  read GOAL_X GOAL_Y GOAL_YAW_DEG <<< "${POSES[$POSE_NAME]}"
  echo "Using named pose '$POSE_NAME': x=$GOAL_X y=$GOAL_Y yaw=$GOAL_YAW_DEG°"
elif [ $# -eq 3 ]; then
  GOAL_X=$1
  GOAL_Y=$2
  GOAL_YAW_DEG=$3
else
  echo "Usage (named):    ./dock_pos.sh <pose_name>"
  echo "Usage (explicit): ./dock_pos.sh <x> <y> <yaw_degrees>"
  echo ""
  echo "Available poses:"
  for name in "${!POSES[@]}"; do
    echo "  $name -> ${POSES[$name]}"
  done
  exit 1
fi

# Convert yaw degrees to radians for quaternion: z = sin(yaw/2), w = cos(yaw/2)
GOAL_YAW_Z=$(python3 -c "import math; print(math.sin(math.radians($GOAL_YAW_DEG)/2))")
GOAL_YAW_W=$(python3 -c "import math; print(math.cos(math.radians($GOAL_YAW_DEG)/2))")

echo "Goal: x=$GOAL_X  y=$GOAL_Y  yaw=${GOAL_YAW_DEG}° (z=$GOAL_YAW_Z, w=$GOAL_YAW_W)"

READY_FLAG=/tmp/stretch_nodes_ready
DOCKED_FLAG=/tmp/stretch_docked

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

# ── Reset arm position ─────────────────────────────────────────────────────
echo "Resetting arm position..."
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger {}
sleep 1

# python3 ~/test1.py; EXIT_CODE=$?
# if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 139 ]; then
#   echo "ERROR: test1.py failed (exit $EXIT_CODE). Aborting."
#   exit 1
# fi
# echo "Arm reset complete (exit $EXIT_CODE)."

ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}

# ── Navigate to goal ───────────────────────────────────────────────────────
echo "=== Navigating to x=$GOAL_X y=$GOAL_Y yaw=${GOAL_YAW_DEG}° ==="
NAV_OUTPUT=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $GOAL_YAW_Z, w: $GOAL_YAW_W}
    }
  }
}" 2>&1)
echo "$NAV_OUTPUT"

if echo "$NAV_OUTPUT" | grep -q "Goal finished with status: SUCCEEDED"; then
  echo "Navigation succeeded!"
else
  echo "ERROR: Navigation did not succeed. Aborting."
  exit 1
fi

# ── Signal that robot is docked ────────────────────────────────────────────
touch "$DOCKED_FLAG"
echo "=== dock_pos.sh finished at $(date) ==="
