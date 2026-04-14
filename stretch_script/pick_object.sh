#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash
LOG_FILE=~/stretch_script/logs/pick_object_$(date +%Y%m%d_%H%M%S).log
mkdir -p ~/stretch_script/logs
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== pick_object.sh started at $(date) ==="

# ── Parse arguments ────────────────────────────────────────────────────────
MARKER_ID=${1:-202}
echo "Target ArUco marker ID: $MARKER_ID"

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

# ── Set marker ID on the running ArUco detector ────────────────────────────
echo "Setting target marker ID to $MARKER_ID..."
ros2 param set /aruco_base_frame_estimator target_marker_id $MARKER_ID

# ── Reset arm so camera has clear view for detection ──────────────────────
echo "Resetting arm position for camera view..."
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger {}
sleep 1
python3 ~/test1.py; EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 139 ]; then
  echo "ERROR: Arm reset failed (exit $EXIT_CODE). Aborting."
  exit 1
fi
echo "Arm reset complete."
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger {}

# ── Wait for a LIVE marker detection (stamp must be moving) ───────────────
# The ArUco node latches its last known pose and keeps republishing it even
# when the marker is out of view — same stamp.sec repeating is the tell.
# We read two messages and require the stamp to have advanced, confirming
# the camera is actively seeing the marker right now.
echo "Waiting for live marker detection (confirming stamp is advancing)..."
LIVE_TIMEOUT=20
LIVE_COUNT=0
while true; do
  LIVE_COUNT=$((LIVE_COUNT + 1))
  if [ $LIVE_COUNT -gt $LIVE_TIMEOUT ]; then
    echo "ERROR: Could not confirm live marker detection after ${LIVE_TIMEOUT}s."
    echo "       Is marker $MARKER_ID visible to the camera? Aborting."
    exit 1
  fi

  SEC1=$(ros2 topic echo /aruco/target_base_link --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')
  sleep 0.5
  SEC2=$(ros2 topic echo /aruco/target_base_link --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')

  if [ -n "$SEC1" ] && [ -n "$SEC2" ] && [ "$SEC2" -gt "$SEC1" ]; then
    echo "Live detection confirmed! (stamp advancing: $SEC1 → $SEC2)"
    # Grab the confirmed live pose for logging
    LIVE_POSE=$(ros2 topic echo /aruco/target_base_link --once 2>/dev/null)
    echo "Live pose:"
    echo "$LIVE_POSE" | grep -E "x:|y:|z:" | head -3
    break
  else
    echo "Attempt $LIVE_COUNT: stamp not advancing yet ($SEC1 / $SEC2) — marker may not be visible"
  fi
done

# ── Wait for grasp service ─────────────────────────────────────────────────
echo "Waiting for grasp service..."
until ros2 service list 2>/dev/null | grep -q "/grasp_object"; do
  sleep 1
done
echo "Grasp service ready!"

# ── Trigger grasp ──────────────────────────────────────────────────────────
echo "=== Triggering grasp ==="
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger {}
sleep 2

GRASP_RESULT=$(ros2 service call /grasp_object std_srvs/srv/Trigger {})
echo "$GRASP_RESULT"
if echo "$GRASP_RESULT" | grep -q "success=True"; then
  echo "Grasp succeeded!"
else
  echo "ERROR: Grasp failed."
  exit 1
fi

echo "=== pick_object.sh finished at $(date) ==="
