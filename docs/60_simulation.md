# Simulating TurtleBot3 with Gazebo


## 1. Background

### What is Gazebo?

Gazebo is an open-source robot simulator that lets you test and develop robot code without physical hardware — useful for prototyping, debugging, and CI/CD pipelines.

There are two generations in active use:

| Version | Engine | ROS 2 Topic Integration |
|---|---|---|
| **Gazebo Classic** (Ignition Citadel/Fortress) | Ignition | Topics like `/cmd_vel`, `/scan`, `/odom` appear automatically as ROS 2 topics. No remapping needed. |
| **Gazebo Sim** (new) | Gz | Uses internal topic names (e.g. `/world/{name}/model/...`). Must be bridged to ROS 2 via `ros_gz_bridge`. |

Here, we use **Gazebo Classic**.

> **Gazebo Sim bridging example** (for reference only):
> ```python
> Node(
>     package='ros_gz_bridge',
>     executable='parameter_bridge',
>     arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
>     remappings=[('/cmd_vel', '/robot_cmd_vel')]
> )
> ```

---


## 2. VMware Setup (Ubuntu 22.04 VM)

If you are running Gazebo inside a VMware VM, do the following before launching any simulation.

### Install VMware Tools

```bash
sudo apt update
sudo apt install open-vm-tools open-vm-tools-desktop
sudo reboot
```

> You may also need to select **"Upgrade this Virtual Machine"** from the VMware menu if prompted.

### Fix Slow Performance

If Gazebo is very slow or fails to render:

- Increase VM **VRAM to 128 MB** in VMware settings
- Toggle **3D acceleration** on or off (try both)

---


## 3. Setting Up Your ROS 2 Simulation Package

Create your own ROS 2 package for the simulation. A recommended layout:

```text
your_package/
  CMakeLists.txt
  package.xml
  launch/
    sim.launch.py
  models/
    v3_block/           ← copy from sim/aice_sim/models/v3_block/
      meshes/
        v3_block.stl
      model.config
      model.sdf
  worlds/
    world.world         ← copy from sim/aice_sim/worlds/world.world
```

The model files and world are available in `sim/aice_sim/` in this repository.


### Making the `v3_block` Model Findable by Gazebo

> [!IMPORTANT] **Gazebo must be able to find the `v3_block` model before opening any world file.** If it is missing, Gazebo Classic will show a **black screen** because the world references `model://v3_block/...`.

#### Option A: Via your ROS 2 package (recommended)

Add this export to your `package.xml` — it tells Gazebo to search your package's installed `models/` directory automatically at launch time:

```xml
<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
</export>
```

And install the `models/` directory in your `CMakeLists.txt`:

```cmake
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
```

No manual `~/.gazebo/models` copy is needed.

#### Option B: Manual install (no ROS package / quick test)

```bash
mkdir -p ~/.gazebo/models
cp -r sim/aice_sim/models/v3_block ~/.gazebo/models/
```

### Creating Coloured Variants (Red / Blue)

#### Option 1 (recommended): Use the provided `world.world`

The file `sim/aice_sim/worlds/world.world` already contains a **2 m × 2 m** enclosed arena with one red block and one blue block. No extra model files are needed.

```bash
gazebo --verbose sim/aice_sim/worlds/world.world
```

#### Option 2: Create separate model folders manually

```bash
cp -r ~/.gazebo/models/v3_block ~/.gazebo/models/v3_block_red
cp -r ~/.gazebo/models/v3_block ~/.gazebo/models/v3_block_blue
```

Then for each copy:

1. Update `<name>` in `model.config` to match the folder name (`v3_block_red` / `v3_block_blue`).
2. Add a `<material>` tag under `<visual>` in `model.sdf`:
   - Red: `Gazebo/Red`
   - Blue: `Gazebo/Blue`

Restart Gazebo after making edits.

---


## 4. Launching a Simulation

### Quick launch (built-in TurtleBot3 world)

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Launch with a custom world file (no ROS package)

```bash
gazebo --verbose /path/to/world.world
```

If you hit loading errors, check the terminal output for lines like `model://v3_block ... not found`.

### ROS 2 Launch file (Gazebo + TurtleBot3 spawn)

Create `launch/sim.launch.py` in your package. This starts Gazebo with your world, publishes the robot description, and spawns a TurtleBot3 burger:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share     = get_package_share_directory('your_package_name')  # change this
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='0.0')
    y_pose       = LaunchConfiguration('y_pose',       default='0.0')

    world_file = os.path.join(pkg_share, 'worlds', 'world.world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose',       default_value='0.0'),
        DeclareLaunchArgument('y_pose',       default_value='0.0'),

        # Start Gazebo server with the arena world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file, 'extra_gazebo_args': '--verbose'}.items(),
        ),

        # Start Gazebo GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Publish robot description (/tf)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Spawn TurtleBot3 into the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),
    ])
```

Run:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch your_package_name sim.launch.py
```

### Viewing the Camera Feed (Waffle Pi)

The TurtleBot3 Waffle Pi has a camera. Once the simulation is running, you can view its feed using `rqt_image_view`.

**On the VM (or any ROS 2 machine on the same domain):**

```bash
ros2 run rqt_image_view rqt_image_view
```

In the dropdown, select `/camera/image_raw` (or `/camera/image_raw/compressed` if you have image transport plugins installed).

To check available camera topics:

```bash
ros2 topic list | grep camera
```

To check the stream rate and bandwidth:

```bash
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw
```

> **Note:** Raw image topics are high bandwidth (~27 MB/s for 640×480 RGB8 at 30 FPS). If you experience lag, use the compressed topic instead. Install the transport plugins if needed:
> ```bash
> sudo apt install ros-humble-image-transport-plugins
> ```
> Then select `/camera/image_raw/compressed` in `rqt_image_view`.

See `docs/30_camera_setup.md` for full camera setup and optimisation instructions for the physical robot.

---

### Simulation time

For nodes that should follow Gazebo's clock rather than wall time, pass `use_sim_time=True`:

```python
parameters=[{'use_sim_time': True}]
```

Or set it at runtime:

```bash
ros2 param set /<node_name> use_sim_time true
```

---

## 5. Deploying Code to the TurtleBot

Once you have tested in simulation, transfer the ROS 2 workspace to the physical robot using one of the options below.

### Option A: Git over HTTPS with a Personal Access Token (PAT)

On the TurtleBot:

```bash
git clone https://github.com/OWNER/REPO.git
# username: your GitHub username
# password: your PAT (not your GitHub password)
```

To update later:

```bash
cd REPO && git pull
```

### Option B: GitHub CLI (`gh`)

On the TurtleBot:

```bash
sudo apt update && sudo apt install gh
gh auth login
gh repo clone OWNER/REPO
```

### Option C: `scp` (one-off copy)

```bash
scp -r ~/my_ros2_ws turtlebot@TURTLEBOT_IP:~
```

### Option D: `rsync` (recommended for repeated transfers)

```bash
rsync -av --delete ~/my_ros2_ws/ <username>@<turtlebot_tailscale_ip>:~/my_ros2_ws/
```

`rsync` only sends changed files, making updates much faster than `scp`.

### After copying: build and source on the TurtleBot

```bash
cd ~/my_ros2_ws
colcon build --executor sequential
source install/setup.bash
```

---

## 6. Reference: `aice_sim` Package

If you are stuck, a complete working package is provided at `sim/aice_sim/` in this repository. It implements everything described in sections 3 and 4 and can be built directly with colcon.

```text
sim/aice_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── sim.launch.py
├── models/
│   └── v3_block/
│       ├── meshes/v3_block.stl
│       ├── model.config
│       └── model.sdf
└── worlds/
    └── world.world
```

### Building

```bash
# From the repo root
colcon build --base-paths sim/aice_sim

# Or place it under a workspace src/ directory
cp -r sim/aice_sim ~/my_ros2_ws/src/
cd ~/my_ros2_ws && colcon build --packages-select aice_sim
```

### Running

```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch aice_sim sim.launch.py
```
