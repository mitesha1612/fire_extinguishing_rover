#!/bin/bash

# ============================================================
# Autonomous Fire-Fighting Rover
# ROS 2 Jazzy Simulation Launcher
# ============================================================

set -e

echo "=============================================="
echo "   FIRE ROVER SIMULATION"
echo "=============================================="

source /opt/ros/jazzy/setup.bash
source ~/fire_rover_ws/install/setup.bash

# Give this launch its own ROS and Gazebo namespaces so stale or separate
# Fire Rover runs cannot claim its controller manager or Gazebo services.
ROS_DOMAIN_ID=$((20 + ($$ % 200)))
GZ_PARTITION="fire_rover_${ROS_DOMAIN_ID}_$$"
export ROS_DOMAIN_ID GZ_PARTITION

echo "[1/4] ROS 2 environment loaded (domain $ROS_DOMAIN_ID)"
echo "Gazebo partition: $GZ_PARTITION"

ROBOT_XACRO=~/fire_rover_ws/src/fire_rover_description/urdf/fire_rover.urdf.xacro
WORLD=~/fire_rover_ws/src/fire_rover_description/worlds/fire_rover.sdf

if [ ! -f "$ROBOT_XACRO" ]; then
    echo "ERROR: Robot Xacro not found:"
    echo "$ROBOT_XACRO"
    exit 1
fi

echo "[2/4] Robot Xacro found"

ROBOT_DESCRIPTION=$(xacro "$ROBOT_XACRO" use_sim:=true)
ROBOT_DESCRIPTION_JSON=$(printf '%s' "$ROBOT_DESCRIPTION" | python3 -c 'import json, sys; print(json.dumps(sys.stdin.read()))')

echo "[3/4] Starting ROS nodes..."

PIDS=()
cleanup() {
    echo ""
    echo "Stopping Fire Rover simulation..."
    local -a PROCESS_TREE=("${PIDS[@]}")
    local -A SEEN=()
    local PID CHILD PARENT CHANGED INDEX

    for PID in "${PIDS[@]}"; do
        SEEN["$PID"]=1
    done

    CHANGED=1
    while [ "$CHANGED" -eq 1 ]; do
        CHANGED=0
        while read -r CHILD PARENT; do
            if [[ -n "${SEEN[$PARENT]+present}" && -z "${SEEN[$CHILD]+present}" ]]; then
                PROCESS_TREE+=("$CHILD")
                SEEN["$CHILD"]=1
                CHANGED=1
            fi
        done < <(ps -eo pid=,ppid=)
    done

    for ((INDEX=${#PROCESS_TREE[@]} - 1; INDEX >= 0; INDEX--)); do
        PID="${PROCESS_TREE[$INDEX]}"
        kill -TERM "$PID" 2>/dev/null || true
    done

    for PID in "${PIDS[@]}"; do
        wait "$PID" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid gz sim -r "$WORLD" &
PIDS+=("$!")

setsid python3 "$HOME/fire_rover_ws/scripts/fire_simulation.py" &
PIDS+=("$!")

setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' &
PIDS+=("$!")

# ------------------------------------------------------------
# Robot State Publisher
# ------------------------------------------------------------
setsid ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p "robot_description:=$ROBOT_DESCRIPTION_JSON" \
    -p use_sim_time:=true &
PIDS+=("$!")

sleep 3

# ------------------------------------------------------------
# Spawn robot and activate its controllers
# ------------------------------------------------------------
ros2 run ros_gz_sim create -name fire_rover -topic robot_description
ros2 run controller_manager spawner joint_state_broadcaster \
    --controller-manager /controller_manager \
    --controller-manager-timeout 60 --service-call-timeout 60
ros2 run controller_manager spawner diff_drive_controller \
    --controller-manager /controller_manager \
    --controller-manager-timeout 60 --service-call-timeout 60

echo ""
echo "=============================================="
echo "   FIRE ROVER SIMULATION RUNNING"
echo "=============================================="
echo ""
echo "Arrow-key controls (hold an arrow to move):"
echo ""
echo "  Up / Down    = forward / backward"
echo "  Left / Right = rotate"
echo "  Space        = start water spray"
echo "  Any other key = stop spray"
echo "  Ctrl+C        = quit teleop"
echo ""
echo "ROS topics:"
echo "  /cmd_vel"
echo "  /diff_drive_controller/cmd_vel"
echo "  /joint_states"
echo "  /robot_description"
echo "  /tf"
echo "  /tf_static"
echo ""
echo "=============================================="
echo "   KEYBOARD TELEOP ACTIVE"
echo "=============================================="
echo ""

# ------------------------------------------------------------
# Arrow-key Teleoperation
#
# IMPORTANT:
# This must stay in the foreground because it needs
# direct access to the terminal keyboard.
# ------------------------------------------------------------

python3 "$HOME/fire_rover_ws/scripts/arrow_teleop.py"
