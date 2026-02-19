#!/bin/bash
###############################################################################
# Auto Completion + Aliases for TurtleBot3 Gazebo
# This script is meant to be sourced from ~/.bashrc
###############################################################################

# Determine base directory relative to THIS script
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"

# tb3_env.sh path
TB3_ENV="${SCRIPT_DIR}/tb3_env.sh"

# Ensure tb3_env.sh exists and source it
if [[ ! -f "${TB3_ENV}" ]]; then
    echo "[TB3_AUTO] ERROR: tb3_env.sh not found at ${TB3_ENV}"
    return 1
fi

# Source tb3_env.sh to get environment setup and TB3_WS variable
source "${TB3_ENV}"

# TB3_WS is now provided by tb3_env.sh
# Check if TurtleBot3 workspace exists
if [[ ! -d "$TB3_WS" ]]; then
    echo "[TB3_AUTO] WARNING: TurtleBot3 workspace not found at $TB3_WS"
    echo "[TB3_AUTO] Please run setup_u22_tb3.sh to install TurtleBot3"
    return 1
fi

# Get the python executable path from the ROS2 environment
PYTHON_EXEC=$(which python3)
if [[ -z "$PYTHON_EXEC" ]]; then
    echo "[TB3_AUTO] ERROR: Python3 executable not found in PATH"
    return 1
fi

# -------------------------------
# TurtleBot3 Gazebo Aliases
# -------------------------------

# Test TurtleBot3 in empty world
alias test_tb3='ros2 launch turtlebot3_gazebo empty_world.launch.py'

# Launch TurtleBot3 worlds
alias tb3_empty='ros2 launch turtlebot3_gazebo empty_world.launch.py'
alias tb3_world='ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'
alias tb3_house='ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py'

# TurtleBot3 teleoperation
alias tb3_teleop='ros2 run turtlebot3_teleop teleop_keyboard'

# TurtleBot3 poweroff remote robots
# Check script for detailed usage instructions
alias tb3_poweroff="${PYTHON_EXEC} ${SCRIPT_DIR}/tb3_poweroff.py"
alias pwf="${PYTHON_EXEC} ${SCRIPT_DIR}/tb3_poweroff.py"

# -------------------------------
# Gazebo Management Commands
# -------------------------------

# Kill Gazebo processes
tb3_kill() {
    echo "[tb3_kill] Searching for Gazebo processes…"

    local PROC_NAMES=(
        gzserver
        gzclient
        gazebo
        gazebo-11
        spawn_entity.py
    )

    local PIDS=""

    for PROC in "${PROC_NAMES[@]}"; do
        local FOUND=$(pgrep -f "$PROC")
        if [[ -n "$FOUND" ]]; then
            PIDS+=" $FOUND"
        fi
    done

    if [[ -z "$PIDS" ]]; then
        echo "[tb3_kill] No Gazebo-related processes detected."
        return 0
    fi

    echo "[tb3_kill] Found PIDs: $PIDS"
    echo "[tb3_kill] Sending SIGTERM…"
    kill $PIDS 2>/dev/null || true
    sleep 1

    local STILL_RUNNING=""
    for PID in $PIDS; do
        if ps -p "$PID" >/dev/null 2>&1; then
            STILL_RUNNING+=" $PID"
        fi
    done

    if [[ -z "$STILL_RUNNING" ]]; then
        echo "[tb3_kill] All Gazebo processes terminated cleanly."
        return 0
    fi

    echo "[tb3_kill] Force killing with SIGKILL (-9): $STILL_RUNNING"
    kill -9 $STILL_RUNNING 2>/dev/null || true
    echo "[tb3_kill] Done."
}

# Kill all ROS2 and Gazebo processes
tb3_kill_all() {
    tb3_kill

    pkill -9 -f "rclpy"          2>/dev/null || true
    pkill -9 -f "python.*launch" 2>/dev/null || true
    pkill -9 -f "ros2 launch"    2>/dev/null || true
    pkill -9 -f "rclcpp"         2>/dev/null || true
    pkill -9 -f "ros2_node"      2>/dev/null || true

    echo "[tb3_kill_all] Restarting ros2 daemon…"
    ros2 daemon stop 2>/dev/null || true
    ros2 daemon start 2>/dev/null || true
    echo "[tb3_kill_all] Done."
}

# -------------------------------
# Build Commands
# -------------------------------

tb3_build() {
    cd "$TB3_WS"
    colcon build --symlink-install
    source "$TB3_WS/install/setup.bash"
}

tb3_build_pkg() {
    if [[ $# -eq 0 ]]; then
        echo "[tb3_build_pkg] ERROR: No package specified"
        return 1
    fi

    cd "$TB3_WS"
    colcon build --symlink-install --packages-select "$@"
    source "$TB3_WS/install/setup.bash"
}

tb3_rebuild_pkg() {
    if [[ $# -eq 0 ]]; then
        echo "[tb3_rebuild_pkg] ERROR: No package specified"
        return 1
    fi

    cd "$TB3_WS"

    for pkg in "$@"; do
        echo "[tb3_rebuild_pkg] Cleaning $pkg"
        rm -rf "build/$pkg" "install/$pkg"
    done

    colcon build --symlink-install --packages-select "$@"
    source "$TB3_WS/install/setup.bash"
}

tb3_clean() {
    cd "$TB3_WS"
    rm -rf build/ install/ log/
}

tb3_rebuild() {
    tb3_clean
    tb3_build
}

# -------------------------------
# Completion Functions
# -------------------------------

# Package name completion
_tb3_complete_packages() {
    local cur="${COMP_WORDS[COMP_CWORD]}"

    if [[ -d "$TB3_WS/build" ]]; then
        local pkgs=$(ls -1 "$TB3_WS/build" 2>/dev/null)
        COMPREPLY=( $(compgen -W "${pkgs}" -- "${cur}") )
        return 0
    fi

    if command -v colcon >/dev/null 2>&1; then
        local pkgs=$(cd "$TB3_WS" && colcon list --names-only 2>/dev/null)
        COMPREPLY=( $(compgen -W "${pkgs}" -- "${cur}") )
        return 0
    fi

    return 0
}

complete -F _tb3_complete_packages tb3_build_pkg tb3_rebuild_pkg

echo "[TB3_AUTO] TurtleBot3 environment loaded (MODEL=${TURTLEBOT3_MODEL})"
return 0

