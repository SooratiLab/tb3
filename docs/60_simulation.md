# Simulating TurtleBot3 with Gazebo Sim

## 1. Background

The AICE2011 lab uses **ROS 2 Jazzy** and **Gazebo Sim Harmonic**. The whole environment runs in the [AICE2011 container](https://github.com/tgodfrey0/AICE2011_Container), so there is no virtual machine to install or configure.

Gazebo Sim is the current generation of Gazebo. It is not Gazebo Classic:

| Gazebo Classic | Gazebo Sim Harmonic |
|---|---|
| `gazebo` | `gz sim` |
| `gzserver` / `gzclient` | `gz sim` server and GUI |
| `GAZEBO_MODEL_PATH` | `GZ_SIM_RESOURCE_PATH` |
| `gazebo_ros` plugins | `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image` |
| `geometry_msgs/msg/Twist` for `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |

Gazebo Sim uses its own transport topics. The `ros_gz_bridge` translates those topics into ROS 2 topics such as `/scan`, `/odom`, `/imu`, and `/camera/image_raw`.

## 2. Start the container

Follow the [container README](https://github.com/tgodfrey0/AICE2011_Container) to start the lab and open the noVNC desktop. Everything below is typed **inside the container**, in a terminal on that desktop.

You do not need to install ROS 2, Gazebo, or TurtleBot3 packages yourself.

### Built-in worlds

The container has a launcher and two ready-made worlds:

```bash
ros2 launch /home/student/lab/launch/tb3_launch.py world:=base_world
ros2 launch /home/student/lab/launch/tb3_launch.py world:=house_world
```

Or use the aliases:

```bash
tb3_base
tb3_house
```

- **Base World** — ground plane and sun, no robot. Spawn a Waffle Pi yourself.
- **TB3 House World** — the TurtleBot3 house with a Waffle Pi already loaded.

To stop a world, press `Ctrl+C` in the terminal that started it, or run `tb3_kill`.

## 3. Use the `aice_sim` package

The reference package is [`sim/aice_sim`](../sim/aice_sim/). It contains the launch file, worlds, and the `v3_block` model.

### Package layout

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
    ├── arena.world
    ├── base_world.world
    └── house_world.world
```

### Build it

Copy the package into your container workspace and build it:

```bash
cp -r /path/to/tb3_guides/sim/aice_sim ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select aice_sim
source install/setup.bash
```

If you have already copied the repository into `~/ros2_ws/src`, build it from there instead. The container mounts `~/ros2_ws` from your computer, so the package survives a container restart.

### Run it

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch aice_sim sim.launch.py world:=arena
```

Other worlds:

```bash
ros2 launch aice_sim sim.launch.py world:=base_world
ros2 launch aice_sim sim.launch.py world:=house_world
```

Run without the Gazebo GUI:

```bash
ros2 launch aice_sim sim.launch.py world:=arena gui:=false
```

The launch file starts the Gazebo server and GUI, publishes the Waffle Pi's TF frames, and bridges the robot topics into ROS 2. The robot is included in the arena and house worlds, so no separate spawn step is needed.

## 4. The arena

`arena.world` is a 2 m × 2 m arena with 1 m high walls. It contains:

- a TurtleBot3 Waffle Pi at the origin;
- a red `v3_block` at `(0.5, 0, 0)`;
- a blue `v3_block` at `(-0.5, 0, 0)`.

The block mesh is stored in millimetres, so the world scales it by `0.001`.

### Adding another block

Copy an existing block model inside the world, give it a unique name, and change its pose:

```xml
<model name="v3_block_extra">
  <static>false</static>
  <pose>0.5 0.5 0 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>0.045</mass>
      <inertia>
        <ixx>2.7e-5</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>2.7e-5</iyy>
        <iyz>0</iyz>
        <izz>2.7e-5</izz>
      </inertia>
    </inertial>
    <visual name="visual">
      <pose>0 0 0.03 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://v3_block/meshes/v3_block.stl</uri>
          <scale>0.001 0.001 0.001</scale>
        </mesh>
      </geometry>
      <material>
        <ambient>0.1 0.5 0.1 1</ambient>
        <diffuse>0.2 0.7 0.2 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
    <collision name="collision">
      <pose>0 0 0.03 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://v3_block/meshes/v3_block.stl</uri>
          <scale>0.001 0.001 0.001</scale>
        </mesh>
      </geometry>
    </collision>
  </link>
</model>
```

Pose format is `x y z roll pitch yaw`.

## 5. Create your own world

Start from `arena.world` or `base_world.world` and save a copy in your own package:

```bash
mkdir -p ~/ros2_ws/src/my_package/worlds
cp sim/aice_sim/worlds/arena.world \
  ~/ros2_ws/src/my_package/worlds/my_world.world
```

Run it by passing the absolute path:

```bash
ros2 launch aice_sim sim.launch.py \
  world:=$HOME/ros2_ws/src/my_package/worlds/my_world.world
```

The launch file sets `GZ_SIM_RESOURCE_PATH` to include:

- `aice_sim/models/`, so `model://v3_block` works;
- the TurtleBot3 models, so `model://turtlebot3_waffle_pi` and the house work;
- anything already in `GZ_SIM_RESOURCE_PATH`.

If your world uses models from your own package, add that model directory before launching:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/my_package/models:$GZ_SIM_RESOURCE_PATH"
ros2 launch aice_sim sim.launch.py \
  world:=$HOME/ros2_ws/src/my_package/worlds/my_world.world
```

### Spawning into the base world

`base_world` deliberately contains no robot. Spawn a Waffle Pi with:

```bash
ros2 run ros_gz_sim create \
  -file /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.1
```

Gazebo reports `Entity creation successful` when it works. If the robot does not appear, wait for the world to finish loading and run the command again.

## 6. Driving and inspecting the robot

Keyboard teleoperation:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

| Key | Action |
|---|---|
| `w` / `x` | Speed up / slow down |
| `a` / `d` | Turn left / right |
| `s` or space | Stop |

Publish a velocity directly. Jazzy uses `TwistStamped`:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.5}}}"
```

Useful checks:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 node list
```

| Topic | Type |
|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/clock` | `rosgraph_msgs/msg/Clock` |

## 7. Camera feed

The Waffle Pi has a simulated camera. View it inside the container:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw`.

To use the compressed topic, install the image transport plugins:

```bash
sudo apt install ros-jazzy-image-transport-plugins
```

Then select `/camera/image_raw/compressed` in `rqt_image_view`. Raw images are high bandwidth, so the compressed topic is useful when the container is slow.

See [Camera Setup](30_camera_setup.md) for the physical camera.

## 8. Simulation time

Nodes that should follow Gazebo's clock rather than wall time need `use_sim_time`:

```python
parameters=[{'use_sim_time': True}]
```

Or at runtime:

```bash
ros2 param set /<node_name> use_sim_time true
```

The `aice_sim` launch file already starts `robot_state_publisher` with simulation time enabled.

## 9. Deploy code to the TurtleBot

Once your package works in simulation, transfer the workspace to the physical Waffle Pi.

### Option A: Git over HTTPS with a Personal Access Token

On the robot:

```bash
git clone https://github.com/OWNER/REPO.git
# username: your GitHub username
# password: your PAT (not your GitHub password)
```

To update later:

```bash
cd REPO && git pull
```

### Option B: GitHub CLI

```bash
sudo apt update && sudo apt install gh
gh auth login
gh repo clone OWNER/REPO
```

### Option C: `scp`

```bash
scp -r ~/ros2_ws turtlebot@<robot-ip>:~
```

### Option D: `rsync` (best for repeated transfers)

```bash
rsync -av --delete ~/ros2_ws/ <username>@<robot_tailscale_ip>:~/ros2_ws/
```

### Build on the robot

```bash
cd ~/ros2_ws
colcon build --executor sequential
source install/setup.bash
```

Use `--executor sequential` on the robot to avoid overloading the Pi.

## 10. Reference: `aice_sim`

The complete package is at [`sim/aice_sim/`](../sim/aice_sim/). It is a working example of everything in this guide and can be built directly with `colcon`.

```bash
cp -r sim/aice_sim ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select aice_sim
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch aice_sim sim.launch.py world:=arena
```

If a world or model fails to load, check the launch terminal for `model://... not found` messages and verify `GZ_SIM_RESOURCE_PATH`. See [Setup Troubleshooting](99_setup_troubleshooting.md) for more.
