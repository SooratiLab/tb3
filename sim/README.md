# AICE2011 - TurtleBot3 Simulation Environment

This repository contains automated setup scripts and tools for AICE2011 TurtleBot3 simulation environments using ROS2 Humble and Gazebo.

## Quick Start

### Option 1: Prebuilt Virtual Machine (OVF) - Fastest

A prebuilt Ubuntu 22.04 VM with ROS2 Humble, Gazebo, and TurtleBot3 already installed is available:

**Download:**
- **Google Drive:** [aice2011_ovf](https://drive.google.com/drive/folders/1HmuMpw_ZSBALqlwnA3RN4CIKW0GDqEab?usp=sharing)
- **OneDrive:** [aice2011_ovf](https://sotonac-my.sharepoint.com/:f:/g/personal/aoa1v22_soton_ac_uk/IgBi6TNXRL7oSZ8iZ4xMT0ogAR8L1MAPgPKF4_JAYxOdcKA?e=Sor9ci)

**Link Valid until:** March 31, 2026

**What's included:**
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo simulator
- TurtleBot3 workspace configured at `~/turtlebot3_ws`
- Python virtual environment at `~/envs/aice2011`
- All TurtleBot3 commands and aliases ready to use

**How to use:**
1. Import the `aice2011.ovf` file into VMWare or VirtualBox
2. Start up the VM and login if needed
3. Open a terminal and run `test_tb3` to launch TurtleBot3 simulation
4. Run `tb3_teleop` in another terminal to control the Turtlebot
5. Run `pwf` to remotely shut down robots when done

**VM Credentials:**
- Username: `aice`
- Password: `2011`

### Option 2: Manual Setup on Ubuntu 22.04

<details>
<summary>Expand for manual setup instructions</summary>

For a complete TurtleBot3 simulation environment on Ubuntu 22.04:

```bash
wget https://git.soton.ac.uk/aoa1v22/aice2011/-/raw/main/scripts/setup_u22_tb3.sh
chmod +x setup_u22_tb3.sh
bash setup_u22_tb3.sh
```

This will install:
- ROS2 Humble
- Gazebo simulator
- TurtleBot3 packages and simulation workspace (`~/turtlebot3_ws`)
- Python virtual environment (`aice2011` in `~/envs/aice2011`)
- TurtleBot3 environment and auto-completion scripts (sourced automatically from `~/.bashrc`)

</details>

### Option 3: ROS2 Humble with Gazebo Only (Minimal)

<details>
<summary>Expand for minimal setup instructions</summary>

```bash
wget https://git.soton.ac.uk/aoa1v22/aice2011/-/raw/main/scripts/setup_u22_gazebo.sh
chmod +x setup_u22_gazebo.sh
bash setup_u22_gazebo.sh
```

Installs ROS2 Humble, Gazebo, and the Python virtual environment — without the TurtleBot3 workspace.

</details>

### Testing Your Installation

> **Note:** Keep your local copy of this repository up to date before each session:
> ```bash
> cd ~/aice2011 && git pull
> ```

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

---

## Setup Scripts (`scripts/`)

- **`setup_u22_tb3.sh`** - Complete TurtleBot3 setup (recommended)
- **`setup_u22_gazebo.sh`** - Minimal ROS2 + Gazebo setup
- **`setup/ros.sh`** - ROS2 Humble installation
- **`setup/gazebo.sh`** - Gazebo configuration
- **`setup/tb3.sh`** - TurtleBot3 workspace setup

---

## Environment Scripts

### `tb3_env.sh` — TurtleBot3 Environment Loader

Sourced automatically by `auto_comp.sh` (and therefore by `~/.bashrc`). Sets up all environment variables required to run TurtleBot3 simulations:

- Sources `/opt/ros/humble/setup.bash` and `/usr/share/gazebo/setup.sh`
- Sources the built TurtleBot3 workspace at `~/turtlebot3_ws/install/setup.bash`
- Sets `TURTLEBOT3_MODEL=waffle_pi`
- Sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Sets `ROS_DOMAIN_ID=30` **if not already defined** — export this variable before sourcing to isolate multiple simultaneous instances
- Sets `GAZEBO_MASTER_URI` based on `ROS_DOMAIN_ID` (default: `http://localhost:11345`)
- Exports `GAZEBO_MODEL_PATH`, `GAZEBO_RESOURCE_PATH`, and `GAZEBO_PLUGIN_PATH`

To override the domain ID for a specific session:
```bash
export ROS_DOMAIN_ID=31
source ~/aice2011/scripts/auto_comp.sh
```

### `auto_comp.sh` — Aliases and Shell Completion

Sourced from `~/.bashrc`. Loads `tb3_env.sh` and registers the following commands and tab-completion functions.

#### Launch Simulations

| Command | Description |
|---|---|
| `test_tb3` | Launch TurtleBot3 in an empty Gazebo world |
| `tb3_empty` | Launch TurtleBot3 in an empty Gazebo world |
| `tb3_world` | Launch TurtleBot3 world |
| `tb3_house` | Launch TurtleBot3 house world |

#### Robot Control

| Command | Description |
|---|---|
| `tb3_teleop` | Control the robot with keyboard (W/A/S/D/X) |
| `pwf` | Remotely shut down TurtleBot3 robots (see below) |
| `tb3_poweroff` | Alias for `pwf` |

#### Process Management

| Command | Description |
|---|---|
| `tb3_kill` | Gracefully stop Gazebo processes (SIGTERM then SIGKILL) |
| `tb3_kill_all` | Stop all Gazebo and ROS2 processes and restart the ROS2 daemon |

#### Workspace Build Commands

| Command | Description |
|---|---|
| `tb3_build` | Build the entire workspace sequentially |
| `tb3_build_par` | Build the entire workspace in parallel |
| `tb3_build_pkg <pkg>` | Build a specific package sequentially |
| `tb3_build_pkg_par <pkg>` | Build a specific package in parallel |
| `tb3_rebuild_pkg <pkg>` | Clean and rebuild a specific package sequentially |
| `tb3_clean` | Remove all build, install, and log artifacts |
| `tb3_rebuild` | Clean then rebuild the entire workspace sequentially |

Tab-completion is available for `tb3_build_pkg`, `tb3_build_pkg_par`, and `tb3_rebuild_pkg` — package names are resolved from the workspace `build/` directory.

### `tb3_poweroff.py` — Remote Robot Shutdown

Shuts down one or more physical TurtleBot3 robots over SSH via Tailscale. Automatically detects valid SSH usernames and caches them in `.quick.csv` for faster subsequent runs. Passwordless (key-based) connections are prioritised over password-based ones.

Common SSH usernames tried during auto-detection: `ubuntu`, `tb`, `pi`, `group`, `turtlebot` (plus the hostname itself). To add custom usernames, edit `COMMON_SSH_USERS` in `tb3_poweroff.py`.

> **⚠ Important:** You must verify that `pwf` successfully shuts down your robot from the VM before running any test on the robot.

#### Usage

```bash
pwf 1              # Shutdown tb1
pwf 1 2 3          # Shutdown tb1, tb2, tb3
pwf                # Auto-detect and shutdown all online TurtleBots
pwf humble         # Shutdown device named 'humble'
pwf -p mypassword  # Provide sudo password
pwf -u ubuntu      # Force a specific SSH username
pwf -d             # Dry-run: list targets without shutting down
pwf -v             # Verbose: show SSH detection details
pwf -q             # Quiet: suppress detailed output
```

> **Important:** Multiple robots are shut down in parallel. The script requires Tailscale to be running and the target devices to be online and reachable.

---

## Python Environment (`venv/`)

- **`create_env.sh`** - Creates Python virtual environment at `~/envs/aice2011`
- **`gen_requirements.py`** - Generates requirements files
- **`requirements/`** - Python dependencies for different Python versions

---

## Installation Locations

- **aice2011 repository**: `~/aice2011`
- **TurtleBot3 workspace**: `~/turtlebot3_ws`
- **Python environment**: `~/envs/aice2011`

## Requirements

- Ubuntu 22.04
- Internet connection
- ~5GB disk space
- Sudo privileges

## Repository Structure

```
aice2011/
├── README.md                  # This file
├── scripts/                   # Setup and management scripts
│   ├── setup/                 # Individual component setup scripts
│   ├── setup_u22_tb3.sh       # Complete TurtleBot3 setup
│   ├── setup_u22_gazebo.sh    # Minimal Gazebo setup
│   ├── auto_comp.sh           # Bash aliases and completion
│   ├── tb3_env.sh             # Environment configuration
│   └── tb3_poweroff.py        # Remote robot shutdown
├── venv/                      # Python environment tools
│   ├── create_env.sh
│   ├── gen_requirements.py
│   └── requirements/
└── setup.py
```

## Troubleshooting

If a script fails:

1. **Check internet connection** - Scripts need to download packages and clone repositories
2. **Ensure sufficient disk space** - ROS2 and simulators require several GB
3. **Review error messages** - Scripts will indicate which component failed
4. **Re-run the script** - Many transient errors can be resolved by running again

## Support

For issues or questions:
1. Ensure all installation steps completed successfully
2. Verify ROS2 and Gazebo are properly installed: `ros2 --version` and `gazebo --version`
3. Report issues at: https://git.soton.ac.uk/aoa1v22/aice2011/-/issues
