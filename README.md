# COMP2011 - TurtleBot3 Simulation Environment

This repository contains automated setup scripts and tools for COMP2011 TurtleBot3 simulation environments using ROS2 Humble and Gazebo.

## Quick Start

### One-Command Setup

For a complete TurtleBot3 simulation environment on Ubuntu 22.04:

```bash
wget https://git.soton.ac.uk/aoa1v22/comp2011/-/raw/main/scripts/setup_u22_tb3.sh
chmod +x setup_u22_tb3.sh
bash setup_u22_tb3.sh
```

This will install:
- ROS2 Humble
- Gazebo simulator
- TurtleBot3 packages and simulation workspace
- Python virtual environment (comp2011)
- Bash aliases and auto-completion

### Testing Your Installation

After setup, open a new terminal and run:
```bash
test_tb3
```

This launches TurtleBot3 in an empty Gazebo world.

To test robot movement, open two terminals:
```bash
# Terminal 1: Launch simulation
tb3_empty

# Terminal 2: Control the robot with keyboard
tb3_teleop
```

Use the keyboard controls (W/A/S/D/X) to move the TurtleBot3 around.

## What's Included

### Setup Scripts (`scripts/`)

- **`setup_u22_tb3.sh`** - Complete TurtleBot3 setup (recommended)
- **`setup_u22_gazebo.sh`** - Minimal ROS2 + Gazebo setup
- **`setup/ros.sh`** - ROS2 Humble installation
- **`setup/gazebo.sh`** - Gazebo configuration
- **`setup/tb3.sh`** - TurtleBot3 workspace setup

### Environment Scripts

- **`auto_comp.sh`** - TurtleBot3 aliases and command auto-completion
- **`tb3_env.sh`** - Environment configuration and variables

### Python Environment (`venv/`)

- **`create_env.sh`** - Creates Python virtual environment at `~/envs/comp2011`
- **`gen_requirements.py`** - Generates requirements files
- **`requirements/`** - Python dependencies for different Python versions

## Available Commands

After installation, these commands are available:

### Launch TurtleBot3 Simulations
- `test_tb3` - Launch empty world
- `tb3_empty` - Launch empty world
- `tb3_world` - Launch TurtleBot3 world
- `tb3_house` - Launch house world

### Robot Control
- `tb3_teleop` - Control robot with keyboard (use W/A/S/D/X keys)

### Process Management
- `tb3_kill` - Stop Gazebo processes
- `tb3_kill_all` - Stop all ROS2 and Gazebo processes

### Workspace Building
- `tb3_build` - Build entire workspace
- `tb3_build_pkg <package>` - Build specific package
- `tb3_rebuild_pkg <package>` - Clean and rebuild package
- `tb3_clean` - Remove build artifacts
- `tb3_rebuild` - Clean and rebuild everything

## Installation Locations

- **comp2011 repository**: `~/comp2011`
- **TurtleBot3 workspace**: `~/turtlebot3_ws`
- **Python environment**: `~/envs/comp2011`

## Requirements

- Ubuntu 22.04
- Internet connection
- ~5GB disk space
- Sudo privileges (unless running in Docker)

## Documentation

Detailed setup instructions and troubleshooting can be found in:
- [`scripts/README.md`](scripts/README.md) - Setup script documentation

## Repository Structure

```
comp2011/
├── README.md              # This file
├── scripts/               # Setup and management scripts
│   ├── README.md         # Setup documentation
│   ├── setup/            # Individual component setup scripts
│   ├── setup_u22_tb3.sh  # Complete TurtleBot3 setup
│   ├── setup_u22_gazebo.sh # Minimal Gazebo setup
│   ├── auto_comp.sh      # Bash aliases and completion
│   └── tb3_env.sh        # Environment configuration
├── venv/                 # Python environment tools
│   ├── create_env.sh     # Environment creation script
│   ├── gen_requirements.py
│   └── requirements/     # Python dependencies
└── setup.py              # Python package setup
```

## Support

For issues or questions:
1. Check [`scripts/README.md`](scripts/README.md) for detailed documentation
2. Ensure all installation steps completed successfully
3. Verify ROS2 and Gazebo are properly installed: `ros2 --version` and `gazebo --version`
4. Report issues at: https://git.soton.ac.uk/aoa1v22/comp2011/-/issues
