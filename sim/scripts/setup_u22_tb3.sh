#!/bin/bash
# This script sets up ROS2 Humble, Gazebo and TurtleBot3 by cloning the aice2011 repository
# and using its setup scripts
# Usage: bash setup_u22_tb3.sh

set -e

# Detect if we need sudo (not in Docker or running as root)
if [ "$EUID" -eq 0 ] || ! command -v sudo &> /dev/null; then
    SUDO=""
    echo "[INFO] Running without sudo (Docker or root user)"
else
    SUDO="sudo"
fi

WORKSPACE_DIR="$HOME"
PYTHON_ENV_REPO="https://git.soton.ac.uk/aoa1v22/aice2011.git"
PYTHON_ENV_DIR="$WORKSPACE_DIR/aice2011"

echo "================================"
echo "ROS2 + Gazebo Setup"
echo "Ubuntu 22.04"
echo "================================"
echo "Working directory: $WORKSPACE_DIR"
echo ""

# Update system packages
echo "Updating package lists..."
$SUDO apt update

# Install essential build tools and libraries
echo "Installing essential build tools and libraries..."
$SUDO apt install -y build-essential cmake git wget curl unzip pkg-config \
    libssl-dev libffi-dev python3-dev python3-pip python3-venv

# Clone or update aice2011 repository
echo ""
echo "================================"
echo "Setting up aice2011 repository"
echo "================================"

if [ -d "$PYTHON_ENV_DIR/.git" ]; then
    echo "[INFO] aice2011 repository exists, updating..."
    cd "$PYTHON_ENV_DIR"
    git fetch origin
    git pull origin main || echo "[WARNING] Failed to pull updates"
    cd "$WORKSPACE_DIR"
elif [ -d "$PYTHON_ENV_DIR" ]; then
    echo "[WARNING] aice2011 directory exists but is not a git repo, removing..."
    rm -rf "$PYTHON_ENV_DIR"
    echo "Cloning aice2011 repository..."
    git clone "$PYTHON_ENV_REPO" "$PYTHON_ENV_DIR"
else
    echo "Cloning aice2011 repository..."
    git clone "$PYTHON_ENV_REPO" "$PYTHON_ENV_DIR"
fi

echo "[OK] aice2011 repository ready"

# Verify required setup scripts exist
echo ""
echo "Verifying required setup scripts..."

SETUP_DIR="$PYTHON_ENV_DIR/scripts/setup"
REQUIRED_SCRIPTS=(
    "$SETUP_DIR/ros.sh"
    "$SETUP_DIR/gazebo.sh"
    "$SETUP_DIR/tb3.sh"
)

MISSING_SCRIPTS=()

for script in "${REQUIRED_SCRIPTS[@]}"; do
    if [ ! -f "$script" ]; then
        MISSING_SCRIPTS+=("$script")
    fi
done

if [ ${#MISSING_SCRIPTS[@]} -gt 0 ]; then
    echo ""
    echo "[ERROR] Missing required setup scripts:"
    for script in "${MISSING_SCRIPTS[@]}"; do
        echo "  - $script"
    done
    echo ""
    echo "Please check the aice2011 repository structure."
    exit 1
fi

echo "[OK] All required scripts found"

# Install ROS2 Humble
echo ""
echo "Setting up ROS 2 Humble..."
bash "$SETUP_DIR/ros.sh"

# Install Gazebo
echo ""
echo "Setting up Gazebo..."
bash "$SETUP_DIR/gazebo.sh"

# Install TurtleBot3
echo ""
echo "Setting up TurtleBot3..."
bash "$SETUP_DIR/tb3.sh"

# Create Python virtual environment
echo ""
echo "Setting up Python virtual environment..."
CREATE_ENV_SCRIPT="$PYTHON_ENV_DIR/venv/create_env.sh"
if [ -f "$CREATE_ENV_SCRIPT" ]; then
    bash "$CREATE_ENV_SCRIPT"
    echo "[OK] Python environment created"
else
    echo "[WARNING] create_env.sh not found at: $CREATE_ENV_SCRIPT"
    echo "You can create the Python environment manually later."
fi

# Add auto_comp.sh to bashrc (tb3_env.sh will be sourced internally)
echo ""
echo "Setting up bash environment scripts..."
BASHRC="$HOME/.bashrc"
AUTO_COMP_SCRIPT="$PYTHON_ENV_DIR/scripts/auto_comp.sh"

if [ -f "$AUTO_COMP_SCRIPT" ]; then
    AUTO_COMP_SOURCE="source $AUTO_COMP_SCRIPT"
    if ! grep -q "$AUTO_COMP_SOURCE" "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# TurtleBot3 Auto-completion and Environment" >> "$BASHRC"
        echo "$AUTO_COMP_SOURCE" >> "$BASHRC"
        echo "[OK] auto_comp.sh added to ~/.bashrc"
    else
        echo "[INFO] auto_comp.sh already in ~/.bashrc"
    fi
else
    echo "[INFO] auto_comp.sh not found, skipping"
fi

echo ""
echo "============================================================"
echo "ROS2 + Gazebo + TurtleBot3 Setup Complete!"
echo "============================================================"
echo ""
echo "To test TurtleBot3 simulation (in a new terminal):"
echo "  test_tb3"
echo ""
echo "Control the robot in another terminal with:"
echo "  tb3_teleop"
echo ""
echo "This will launch TurtleBot3 in an empty Gazebo world."
echo ""
echo "Python virtual environment has been created."
echo "To activate it:"
echo "  source ~/envs/aice2011/bin/activate"
echo ""
echo "To use ROS2 manually:"
echo "  source /opt/ros/humble/setup.bash"
echo "============================================================"