#!/bin/bash
# This script sets up TurtleBot3 simulation for ROS2 Humble
# Based on: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
# Creates a turtlebot3 workspace and installs required packages

set -e

# Detect if we need sudo (not in Docker or running as root)
if [ "$EUID" -eq 0 ] || ! command -v sudo &> /dev/null; then
    SUDO=""
    echo "[INFO] Running without sudo (Docker or root user)"
else
    SUDO="sudo"
fi

echo "================================"
echo "TurtleBot3 Simulation Setup"
echo "================================"
echo ""

# Check if ROS 2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "[ERROR] ROS 2 Humble is not installed."
    echo "Please run the ros.sh setup script first."
    exit 1
fi

echo "[INFO] ROS 2 Humble found at /opt/ros/humble"
source /opt/ros/humble/setup.bash
echo ""

# Install TurtleBot3 packages
echo "Installing TurtleBot3 packages..."
echo "  - Updating apt cache..."
$SUDO apt update
echo "  - Installing ros-humble-dynamixel-sdk..."
$SUDO apt install -y ros-humble-dynamixel-sdk
echo "  - Installing ros-humble-turtlebot3-msgs..."
$SUDO apt install -y ros-humble-turtlebot3-msgs
echo "  - Installing ros-humble-turtlebot3..."
$SUDO apt install -y ros-humble-turtlebot3

echo "[OK] TurtleBot3 packages installed"
echo ""

# Determine turtlebot3 workspace location
# If comp2011 exists in home, install turtlebot3 next to it
# Otherwise, install in home directory
if [ -d "$HOME/comp2011" ]; then
    TB3_WS_DIR="$HOME/turtlebot3"
    echo "[INFO] comp2011 found in home, installing turtlebot3 at: $TB3_WS_DIR"
else
    TB3_WS_DIR="$HOME/turtlebot3"
    echo "[INFO] Installing turtlebot3 at: $TB3_WS_DIR"
fi

# Create turtlebot3 workspace directory
echo "Creating workspace directory..."
mkdir -p "$TB3_WS_DIR/src"
cd "$TB3_WS_DIR/src"

echo "TurtleBot3 workspace created at: $TB3_WS_DIR"
echo ""

# Clone TurtleBot3 Simulations repository
echo "Cloning TurtleBot3 Simulations repository..."
if [ -d "turtlebot3_simulations" ]; then
    echo "[INFO] turtlebot3_simulations already exists, pulling latest changes..."
    cd turtlebot3_simulations
    git pull || echo "[WARNING] Failed to pull updates"
    cd "$TB3_WS_DIR/src"
else
    echo "Cloning from GitHub (humble-devel branch)..."
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi

echo "[OK] TurtleBot3 Simulations repository ready"
echo ""

# Build the workspace
echo "Building TurtleBot3 workspace..."
cd "$TB3_WS_DIR"

# Check if colcon is installed
if ! command -v colcon &> /dev/null; then
    echo "[INFO] Installing colcon build tools..."
    $SUDO apt install -y python3-colcon-common-extensions
fi

echo "Running colcon build (this may take a few minutes)..."
colcon build --symlink-install

echo "[OK] Workspace built successfully"
echo ""

# Set up environment variables in bashrc
BASHRC="$HOME/.bashrc"

# Add TURTLEBOT3_MODEL environment variable
if ! grep -q "export TURTLEBOT3_MODEL" "$BASHRC"; then
    echo "Adding TURTLEBOT3_MODEL to ~/.bashrc..."
    echo "" >> "$BASHRC"
    echo "# TurtleBot3 Configuration" >> "$BASHRC"
    echo "export TURTLEBOT3_MODEL=burger" >> "$BASHRC"
    echo "[OK] TURTLEBOT3_MODEL added to ~/.bashrc"
else
    echo "[INFO] TURTLEBOT3_MODEL already in ~/.bashrc"
fi

# Add workspace source to bashrc
TB3_SETUP_LINE="source $TB3_WS_DIR/install/setup.bash"
if ! grep -q "$TB3_SETUP_LINE" "$BASHRC"; then
    echo "Adding TurtleBot3 workspace to ~/.bashrc..."
    echo "$TB3_SETUP_LINE" >> "$BASHRC"
    echo "[OK] TurtleBot3 workspace added to ~/.bashrc"
else
    echo "[INFO] TurtleBot3 workspace already in ~/.bashrc"
fi

# Export for current session
export TURTLEBOT3_MODEL=burger
source "$TB3_WS_DIR/install/setup.bash"

echo ""
echo "================================"
echo "✓ TurtleBot3 Setup Complete!"
echo "================================"
echo ""
echo "Installed components:"
echo "  - TurtleBot3 ROS packages"
echo "  - TurtleBot3 Simulations (Gazebo)"
echo "  - Workspace at: $TB3_WS_DIR"
echo ""
echo "Environment variables set:"
echo "  - TURTLEBOT3_MODEL=burger"
echo ""
echo "To launch TurtleBot3 simulation:"
echo "  ros2 launch turtlebot3_gazebo empty_world.launch.py"
echo ""
echo "Available worlds:"
echo "  - empty_world.launch.py"
echo "  - turtlebot3_world.launch.py"
echo "  - turtlebot3_house.launch.py"
echo ""
echo "To test manually in a new terminal:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  source ~/turtlebot3/install/setup.bash"
echo "  ros2 launch turtlebot3_gazebo empty_world.launch.py"
echo "================================"

