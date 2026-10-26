# AICE2011 TurtleBot3 Simulation

The TurtleBot3 simulation runs in the **AICE2011 container**: ROS 2 Jazzy, Gazebo Sim Harmonic, and a Linux desktop reached through your browser. There is no virtual machine to download or set up.

The container is maintained separately at [tgodfrey0/AICE2011_Container](https://github.com/tgodfrey0/AICE2011_Container). Start it by following that repository's README, then use the terminal on the container desktop.

The robot model is always the **TurtleBot3 Waffle Pi** (`waffle_pi`).

## Quick start inside the container

The container ships two worlds and a launcher:

```bash
ros2 launch /home/student/lab/launch/tb3_launch.py world:=base_world
ros2 launch /home/student/lab/launch/tb3_launch.py world:=house_world
```

As a terminal inside the container opens, these aliases are available:

| Command | What it does |
|---|---|
| `tb3_base` / `tb3_empty` / `test_tb3` | Start the empty base world |
| `tb3_house` | Start the TurtleBot3 house world |
| `tb3_teleop` | Drive the robot with the keyboard |
| `tb3_spawn` | Spawn a Waffle Pi into the base world |
| `tb3_kill` | Stop Gazebo |
| `tb3_kill_all` | Stop Gazebo and every ROS 2 node |
| `tb3_rviz2` | Start RViz2 |
| `tb3_build` | Build `~/ros2_ws` sequentially |
| `tb3_build_pkg <package>` | Build one package |

The full command reference is also available in the container at **Applications → Command Reference**.

## Using the `aice_sim` package

[`aice_sim`](aice_sim/) is the reference ROS 2 package for this repository. It contains:

| World | Description |
|---|---|
| `base_world` | Ground plane and sun. No robot. |
| `house_world` | The TurtleBot3 house with a Waffle Pi already loaded. |
| `arena` | A 2 m × 2 m walled arena with a Waffle Pi and red/blue v3 blocks. |

### Build

Copy the package into your container workspace and build it:

```bash
cp -r /path/to/tb3_guides/sim/aice_sim ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select aice_sim
source install/setup.bash
```

### Run

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch aice_sim sim.launch.py world:=arena
```

You can also run the other worlds:

```bash
ros2 launch aice_sim sim.launch.py world:=base_world
ros2 launch aice_sim sim.launch.py world:=house_world
```

Pass an absolute path for your own world:

```bash
ros2 launch aice_sim sim.launch.py world:=$HOME/ros2_ws/src/my_package/worlds/my_world.world
```

The launch file starts the Gazebo server and GUI, publishes the robot's TF frames, and bridges Gazebo topics to ROS 2. Set `gui:=false` to run without the Gazebo window.

### Topics

The bridge provides:

| ROS 2 topic | Type |
|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

Jazzy uses `TwistStamped` for velocity commands:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.5}}}"
```

Check the simulation:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once
```

View the camera feed:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw`, or install the image transport plugins and use the compressed topic as described in [Camera Setup](../docs/30_camera_setup.md).

## Gazebo Sim notes

This is **Gazebo Sim Harmonic**, not Gazebo Classic:

- The command is `gz sim`, not `gazebo`.
- There are no `gzserver` or `gzclient` processes.
- Models are found through `GZ_SIM_RESOURCE_PATH`, not `GAZEBO_MODEL_PATH`.
- The `aice_sim` launch file adds both the package's `models/` directory and the TurtleBot3 models to `GZ_SIM_RESOURCE_PATH`.
- The container uses Mesa software rendering because there is no GPU. A real-time factor below `1.0` is normal.

To spawn a robot manually into `base_world`:

```bash
ros2 run ros_gz_sim create \
  -file /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.1
```

## Hardware helpers

The scripts in [`scripts/`](scripts/) are for physical robots, not simulation:

- `tb3_poweroff.py` — shut down one or more TurtleBots over Tailscale SSH.
- `terminal.py` — helper used by `tb3_poweroff.py`.

The container provides the ROS 2 and Gazebo environment, so no host-side setup or Python virtual environment is required.

Example remote shutdown:

```bash
python3 sim/scripts/tb3_poweroff.py -d      # dry run
python3 sim/scripts/tb3_poweroff.py 1 2 3   # shut down tb1, tb2 and tb3
```

Tailscale must be running on the machine that executes the script, and the robots must be online and reachable. See [Networking](../docs/20_networking.md) for the Fast DDS over Tailscale setup.

## Troubleshooting

See [Setup Troubleshooting](../docs/99_setup_troubleshooting.md). The most common simulation issues are:

- `/scan` missing: the software renderer is not working. Check `glxinfo -B` for `llvmpipe`.
- A model cannot be found: check `GZ_SIM_RESOURCE_PATH`.
- Stale processes: run `tb3_kill_all` inside the container.
- The simulation is slow: close the Gazebo GUI or use `gui:=false`.

## Support

For issues with the simulation environment, check the command reference inside the container and the [container repository](https://github.com/tgodfrey0/AICE2011_Container). For issues with the TurtleBot3 itself, see the [TB3 guides](../docs/).
