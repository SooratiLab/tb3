#!/bin/bash
###############################################################################
# TurtleBot3 Environment Loader - can be used as required
###############################################################################

# Resolve script path
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"

# Workspace root = parent of parent of this script
BASE_DIR="$(realpath "${SCRIPT_DIR}/../..")"

# TurtleBot3 workspace - always in home directory
TB3_WS="$HOME/turtlebot3_ws"

# Expose to caller
export BASE_DIR
export TB3_WS

# echo "[TB3_ENV] Workspace detected at: ${BASE_DIR}"

# Check workspace
if [[ ! -d "${TB3_WS}" ]]; then
    echo "[TB3_ENV] WARNING: TurtleBot3 workspace not found at ${TB3_WS}"
    echo "[TB3_ENV] Please run setup_u22_tb3.sh to install TurtleBot3"
fi

# ROS_DOMAIN_ID and GZ_PARTITION should be set by the calling environment
# This allows multiple TB3 instances to run simultaneously without interference
if [[ -z "${ROS_DOMAIN_ID}" ]]; then
    export ROS_DOMAIN_ID=30 # Default fallback
    # echo "[TB3_ENV] WARNING: ROS_DOMAIN_ID not set, using default: 30"
fi

if [[ -z "${GAZEBO_MASTER_URI}" ]]; then
    # Default Gazebo master port based on ROS_DOMAIN_ID
    GAZEBO_PORT=$((11345 + ROS_DOMAIN_ID - 30))
    export GAZEBO_MASTER_URI="http://localhost:${GAZEBO_PORT}"
    # echo "[TB3_ENV] WARNING: GAZEBO_MASTER_URI not set, using default: ${GAZEBO_MASTER_URI}"
fi

# Source ROS2
if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
    echo "[TB3_ENV] ERROR: ROS2 Humble not installed."
    return 1
fi

source /opt/ros/humble/setup.bash

# Source Gazebo
if [[ ! -f "/usr/share/gazebo/setup.sh" ]]; then
    echo "[TB3_ENV] ERROR: Gazebo not installed."
    return 1
fi

source /usr/share/gazebo/setup.sh

# Source TurtleBot3 workspace
if [[ -f "${TB3_WS}/install/setup.bash" ]]; then
    source "${TB3_WS}/install/setup.bash"
else
    echo "[TB3_ENV] WARNING: TurtleBot3 workspace not built yet at ${TB3_WS}"
fi

# TB3 Model
export TURTLEBOT3_MODEL=burger

# Gazebo integration
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${TB3_WS}/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH}:${HOME}/.gazebo"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:/opt/ros/humble/lib"

# RMW selection
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[TB3_ENV] OK! (MODEL=${TURTLEBOT3_MODEL})"
return 0
