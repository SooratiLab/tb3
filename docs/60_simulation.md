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

<details>
<summary>Gazebo Sim bridging example (for reference only)</summary>

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    remappings=[('/cmd_vel', '/robot_cmd_vel')]
)
```

</details>

---


## 2. VM Setup (Ubuntu 22.04 VM)

If you are running Gazebo inside a VM, expand the relevant section below before launching any simulation.

<details>
<summary>VMware</summary>

#### Install VMware Tools

```bash
sudo apt update
sudo apt install open-vm-tools open-vm-tools-desktop
sudo reboot
```

> You may also need to select **"Upgrade this Virtual Machine"** from the VMware menu if prompted.

#### Fix Slow Performance

If Gazebo is very slow or fails to render:

- Increase VM **VRAM to 128 MB** in VMware settings
- Toggle **3D acceleration** on or off (try both)

</details>

<details>
<summary>VirtualBox</summary>

#### Install Guest Additions

Guest Additions enables display acceleration and proper rendering inside the VM.

1. With the VM running, go to **Devices → Insert Guest Additions CD image…** in the VirtualBox menu.
2. Run the installer inside the VM:

```bash
sudo apt update
sudo apt install build-essential dkms linux-headers-$(uname -r)
sudo /media/$USER/VBox_GAs_*/VBoxLinuxAdditions.run
sudo reboot
```

#### Fix Slow Performance

If Gazebo is very slow or fails to render:

- Go to **Settings → Display** for the VM and increase **Video Memory to 128 MB**
- Enable **3D Acceleration** under the same Display tab
- Make sure Guest Additions are installed (see above) — without them, 3D acceleration has no effect

> **Note:** VirtualBox 3D acceleration on Linux hosts requires the `vboxvideo` kernel module from Guest Additions. If Gazebo still crashes or shows a black screen, try disabling 3D acceleration and falling back to software rendering with:
> ```bash
> export LIBGL_ALWAYS_SOFTWARE=1
> gazebo
> ```

</details>

---


## 3. Setting Up Your ROS 2 Simulation Package

Your simulation package should live inside a ROS 2 colcon workspace. If you haven't created one yet, see [`50_creating_ros_packages.md`](50_creating_ros_packages.md) first.

Place your package under the workspace `src/` directory:

```text
my_ros2_ws/
  src/
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

The model files and world are available [here](../sim/aice_sim/).

**You will need to download them to your VM or Computer before use**


### Making the `v3_block` Model Findable by Gazebo

> [!IMPORTANT]
> **Gazebo must be able to find the `v3_block` model before opening any world file.** If it is missing, Gazebo Classic will show a **black screen** because the world references `model://v3_block/...`.

You can use either of the following methods to addess this.

#### Option A: Via your ROS 2 package (recommended)

Add this export to your `package.xml` — it tells Gazebo to search your package's installed `models/` directory automatically at launch time:

```xml
<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
</export>
```

And install the `models/`, `worlds/`, and `launch/` directories in your `CMakeLists.txt`:

```cmake
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)
```

### Building Your Package

Run `colcon build` from the **workspace root** (the `my_ros2_ws/` directory containing `src/`). colcon will create `install/`, `build/`, and `log/` there.

```bash
cd ~/my_ros2_ws

# Standard build
colcon build --packages-select your_package_name

# Low-RAM / VM: build one package at a time
colcon build --packages-select your_package_name --executor sequential

# Source the result
source install/setup.bash
```

<details>
<summary>Option B: Manual install (no ROS package / quick test)</summary>

Copy the downloaded `v3_block` files to your `~/.gazebo/models` folder using the following command:

```bash
mkdir -p ~/.gazebo/models
cp -r path/to/downloaded/v3_block ~/.gazebo/models/
```

</details>

### Creating Coloured Variants (Red / Blue)

#### Option 1 (recommended): Use the provided `world.world`

The file `sim/aice_sim/worlds/world.world` already contains:

- A **2 m × 2 m** enclosed wooden-wall arena
- A **red block** (`v3_block_red`) at position `(0.5, 0, 0)`
- A **blue block** (`v3_block_blue`) at position `(-0.5, 0, 0)`

Both blocks use the shared `v3_block` mesh scaled to real units (`scale 0.001`) with a Gazebo material applied (`Gazebo/Red`, `Gazebo/Blue`).

**To open it via your ROS 2 launch file (recommended):**

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch your_package_name sim.launch.py

# You can comment out "'extra_gazebo_args': '--verbose'," to reduce the verbose rate
```

**To open the world directly (standalone Gazebo, Option B model install):**

```bash
gazebo --verbose sim/aice_sim/worlds/world.world

# WARNING: This will fail or show blank if you did not copy the model files to ~/.gazebo/models
```

**Adding more blocks:** open `world.world` in a text editor and duplicate an existing `<model>` block. Give each copy a unique `name` attribute and adjust its `<pose>` (format: `x y z roll pitch yaw`). For example, to add a second red block at `(0.5, 0.5, 0)`:

```xml
<model name="v3_block_red_2">
  <static>1</static>
  <pose>0.5 0.5 0 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <pose>0 0 0.03 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://v3_block/meshes/v3_block.stl</uri>
          <scale>0.001 0.001 0.001</scale>
        </mesh>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Red</name>
        </script>
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

Restart Gazebo after saving the file.

<details>
<summary>Option 2: Create separate model folders manually</summary>

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

</details>

<details>
<summary>Option 3: Edit the world interactively in the Gazebo GUI</summary>

You can place, move, rotate, and scale models directly in the Gazebo GUI without editing any XML.

**Step 1 — Open the world:**

```bash
gazebo --verbose sim/aice_sim/worlds/world.world
```

**Step 2 — Insert a model:**

1. Click the **Insert** tab in the left panel.
2. Navigate to your model path (e.g. `~/.gazebo/models` or your package's `models/` directory).
3. Click `v3_block` — a ghost preview will follow your cursor into the viewport.
4. Click in the scene to place it.

**Step 3 — Transform tools** (toolbar at the top, or keyboard shortcuts):

| Tool | Shortcut | What it does |
|---|---|---|
| **Select** | `Esc` | Select a model to inspect it |
| **Translate** | `T` | Drag the model along X/Y/Z axes using the arrow handles |
| **Rotate** | `R` | Rotate the model around X/Y/Z axes using the ring handles |
| **Scale** | `S` | Resize the model (uniform or per-axis) |

Click a model first to select it (it highlights), then switch to the desired tool. Hold **Ctrl** while dragging to snap to the grid.

**Step 4 — Save the world:**

Go to **File → Save World As** and save over your `world.world` file (or to a new name). This writes the current state — including all model poses — back to SDF.

> **Note:** The GUI scale tool changes the visual size of the instance only; it does not alter the underlying mesh or `model.sdf`. For a permanent size change, edit the `<scale>` value in the `<geometry>` tag inside `world.world` directly.

</details>

<details>
<summary>Quick launch — built-in TurtleBot3 world (no custom arena)</summary>

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

</details>

<details>
<summary>Launch with a custom world file (standalone Gazebo, no ROS package)</summary>

```bash
gazebo --verbose /path/to/world.world
```

If you hit loading errors, check the terminal output for lines like `model://v3_block ... not found`.

</details>

### ROS 2 Launch file (Gazebo + TurtleBot3 spawn)

Create `launch/sim.launch.py` in your package. A complete, working example is available at [`sim/aice_sim/launch/sim.launch.py`](../sim/aice_sim/launch/sim.launch.py) — copy it into your package and replace `'aice_sim'` with your package name.

It starts Gazebo with your arena world, publishes the robot description, and spawns a TurtleBot3 Waffle Pi. Key arguments: `use_sim_time` (default `true`), `x_pose`, `y_pose`.

Run:

```bash
export TURTLEBOT3_MODEL=waffle_pi
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

<details>
<summary>Option B: GitHub CLI (<code>gh</code>)</summary>

On the TurtleBot:

```bash
sudo apt update && sudo apt install gh
gh auth login
gh repo clone OWNER/REPO
```

</details>

<details>
<summary>Option C: <code>scp</code> (one-off copy)</summary>

```bash
scp -r ~/my_ros2_ws turtlebot@TURTLEBOT_IP:~
```

</details>

<details>
<summary>Option D: <code>rsync</code> (recommended for repeated transfers)</summary>

```bash
rsync -av --delete ~/my_ros2_ws/ <username>@<turtlebot_tailscale_ip>:~/my_ros2_ws/
```

`rsync` only sends changed files, making updates much faster than `scp`.

</details>

### After copying: build and source on the TurtleBot

```bash
cd ~/my_ros2_ws
colcon build --executor sequential
source install/setup.bash
```

---

## 6. Reference: `aice_sim` Package

<details>
<summary>Expand — complete working package if you are stuck</summary>

A complete working package is provided at [`sim/aice_sim/`](../sim/aice_sim/) in this repository. It implements everything described in sections 3 and 4 and can be built directly with colcon.

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

</details>
