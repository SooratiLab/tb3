#let version = "v2.2"

#set text(font: "DejaVu Sans", size: 10pt, lang: "en", region: "gb")
#set page(margin: 2cm)
#set par(justify: true)
#set page(numbering: "1")
#set page(header: [
  #set text(8pt)
  #version
])
#show link: set text(fill: blue)
#show link: underline
#set heading(numbering: "1.")

#import "@preview/showybox:2.0.4": showybox
#import "@preview/zebraw:0.6.1": *

#title[TurtleBot3 Setup Guide]
#outline()


= Custom TurtleBot3 Image Builder

== Outline

This tool utilises Packer and Podman to automate the creation of Raspberry Pi 4 images for TurtleBot3 robots. The tool can be found #link("https://github.com/tgodfrey0/turtlebot3_custom_image")[here]. _Disclaimer:_ this system was designed on Linux, YMMV on Windows or MacOS but most instructions are transferable.

A pre-made image for the TB3 Burger (LDS-03, PiCamera (`libcamera` stack)) can be found #link("https://drive.google.com/drive/folders/1bvYBT5LfiV7HOOFlRQBDTyHcVLiAmQ3g?usp=sharing")[here]. Make sure to follow the @post instructions once you have flashed the MicroSD card (see @deploy). For this pre-built image, the username is `tb` and the password is `password`; you should change the password to something else. The image is built with the University of Southampton's IoT network as a placeholder; after booting, edit the file `/etc/netplan/50-cloud-init.yaml` and input your network details.

== Prerequisites
- *Python 3.11+*
- *Podman* (Containerised Packer execution)
- *Dependencies*: `pip install -r requirements.txt`

== Usage Pipeline

=== Configuration
Customise the build via TOML files in `configs/`. Multiple WiFi networks can be defined for robust connectivity.

#zebraw(
  numbering: false,
  indentation: 2,
  ```toml
[model]
type = "waffle" # or "burger"

[ros]
domain_id = 42 

[[network]]
ssid = "Swarm_Lab_5G"
password = "secure_password"
```)


See the `configs/example.toml` to view a complete example.

#showybox(
  frame: (
    border-color: red.darken(50%),
    title-color: red.lighten(60%),
    body-color: red.lighten(80%)
  ),
  title-style: (
    color: black,
    weight: "regular",
    align: center
  ),
  shadow: (
    offset: 3pt,
  ),
  title: "Changing the image size and ISO",
  "It is not recommended to change the ISO that the config points to and the target image size (unless adding new packages/files)."
)

=== Build Process
Run the build script to generate the .img (or compressed .img.xz) file.

#zebraw(
  numbering: false,
  indentation: 2,
```python
python build.py --config configs/my_robot.toml --verbose
```)

The build may take over 1 hour to build and customise the image. 

=== Deployment<deploy>
Flash the image to a MicroSD card. Ensure you identify the correct device node using `fdisk -l`.

#zebraw(
  numbering: false,
  indentation: 2,
```bash
xz -dc <IMAGE>.img.xz | sudo dd of=/dev/<DEVICE> status=progress
```)

After flashing, expand the main partition to fill the MicroSD card. I'd recommend a tool like GParted to expand it to fill the entire MicroSD card.

== Post-Installation<post>

- Log in with the credentials used in your config file
- First Boot: The system automatically configures the hostname (based on MAC address), OpenCR firmware, and Pi Camera. A reboot is required after the first initialisation. *Do not reboot immediately*---wait a few minutes for each process to finish.
- Use the system as normal.

== Key Automated Features

- ROS2 Humble Hawksbill & TurtleBot3 stack installation.
- OpenCR firmware configuration via boot-time service.
- MAC-based Hostname: Automatically sets turtlebot_XX_XX_XX.
- `libcamera` setup for Pi Camera stack
- Netplan configuration for multiple pre-defined SSIDs.

=== Automatic Bringup

This image has a service to automatically run `ros2 launch turtlebot3_bringup robot.launch.py`. To enable this, run the following.

#zebraw(
  numbering: false,
  indentation: 2,
```bash
systemctl enable bringup.service # Run on boot
systemctl start bringup.service # Run now
systemctl status bringup.service # Check it launched with no errors
```)

= Setting up Zenoh with Tailscale for ROS 2 TurtleBot3

== Prerequisite

You should have a working Turtlebot3 Waffle with the correct image. If you do not have this, read the previous section. You will also need a virtual machine to run the remote environment.

You will need a VM to run on the remote PC; you can download VMWare Workstation Pro for Windows #link("https://drive.google.com/file/d/1fImMbF3tlhrv6NMppyDaYYmm7c6O-o8C/view?usp=drive_link")[here].

== Setup Tailscale on VM and TurtleBot3

1. Create a Tailscale account using your Gmail account or GitHub
2. Install Tailscale on the VM:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
curl -fsSL https://tailscale.com/install.sh | sh
```)

3. Start Tailscale to add the VM as a device:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo tailscale up
```)

4. Install Tailscale on the TurtleBot3 and add it to the same Tailscale network.
5. Connect the TurtleBot3 to Tailscale and enable SSH:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo tailscale up --ssh
```)

6. Ensure you connect using the correct username:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
<username>@computername:~$
```)

== Setup Zenoh on VM and TurtleBot3

1. Install Zenoh (RMW implementation) on both devices:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo apt update && sudo apt install ros-humble-rmw-zenoh-cpp
```)

2. Confirm the installation:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 doctor --report | grep rmw
```)

You should see rmw_zenoh_cpp listed.

3. Initialise the ROS 2 environment and set the Zenoh middleware:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
source /opt/ros/humble/setup.sh
source ~/.bashrc
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```)

4. Verify middleware configuration:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 doctor --report | grep middleware
```)

Expected output:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
middleware name: rmw_zenoh_cpp
```)

A warning may appear. This is normal.

== Starting the Zenoh Middleware

One device must run as the router, the other as a client. We recommend running the router on the VM and having the robot as a client.

=== On the router machine

1. Stop any existing ROS services:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
pkill -9 -f ros
ros2 daemon stop
```)

2. Start the Zenoh router:


#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```)

3. Open a new terminal and get the router’s Tailscale IP:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
tailscale ip
```)

=== On the client machine

1. Set the Zenoh client configuration, replacing <router_tailscale_ip> with the router’s Tailscale IP:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<router_tailscale_ip>:7447"]'
```)

2. Confirm connectivity:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 doctor --report | grep middleware
```)

Output should show only:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
middleware name: rmw_zenoh_cpp
```)

== Testing with Talker–Listener

1. Verify the ROS domain ID on both devices:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
echo $ROS_DOMAIN_ID
```)

2. If blank or different, set them to the same value:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
export ROS_DOMAIN_ID=<domain_id_integer>
```)

3. On the router machine:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
source /opt/ros/humble/setup.sh
source ~/.bashrc
ros2 run demo_nodes_cpp talker
```)

4. On the client machine:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 run demo_nodes_cpp listener
```)

Messages should be received.

== Manual Camera Setup

#showybox(
  frame: (
    border-color: red.darken(50%),
    title-color: red.lighten(60%),
    body-color: red.lighten(80%)
  ),
  title-style: (
    color: black,
    weight: "regular",
    align: center
  ),
  shadow: (
    offset: 3pt,
  ),
  title: "Setting Up the Camera Again",
  "If you previously tries to set up the camera, make sure to uninstall all dependencies and remove the `libcamera` repository beforehand."
)

The instructions to set up the camera manually differs slightly from the ROBOTIS instructions. First install the dependencies (make sure not to install Meson or Ninja from `apt`).

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo apt update
sudo apt install -y python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev qtbase5-dev libqt5core5a libqt5widgets5 cmake python3-yaml python3-ply libglib2.0-dev libgstreamer-plugin1.0-dev
sudo apt install ros-humble-camera-ros
pip install meson ninja
```)

Then we can clone the repository and build the package.

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b v0.5.2 https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build -j 2
ninja -C build install -j 2
```)

Then add the following line to you `~/.bashrc` file.

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```)

Restart the robot and test the camera.


== Camera Test

On the Turtlebot:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch turtlebot3_bringup camera.launch.py format:=RGB888 width:=640 height:=480
```)

Recommended formats:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
BGR888 (64x64 - 3280x2464)
RGB888 (64x64 - 3280x2464)
```)

On the Remote PC:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 run rqt_image_view rqt_image_view
```)

Select /camera/image_raw/compressed.

== Image Transport Optimisation

Raw image topics are very high bandwidth (approximately 27 MB/s for 640×480 RGB8 at 30 FPS).

Install plugins:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo apt install ros-humble-image-transport-plugins
```)

Python launch file node:

#zebraw(
  numbering: false,
  indentation: 2,
  ```python
from launch_ros.actions import Node

image_compressed_republisher = Node(
    package='image_transport',
    executable='republish',
    name='image_compressed_republisher',
    arguments=[
        'raw',
        'compressed',
        '--ros-args',
        '-r', 'in:=/camera/image_raw',
        '-r', 'out:=/camera/image_raw/compressed'
    ],
    output='screen'
)

ld.add_action(image_compressed_republisher)
```)

Verify:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 topic list | grep compressed
```)

== Useful ROS 2 Commands

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 topic hz <topic_name>
ros2 topic bw <topic_name>
ros2 topic echo <topic_name> | head
ros2 run rqt_image_view rqt_image_view
ros2 run image_tools showimage --ros-args -r image:=<camera_image_topic>
```)

Always source after changes:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
source install/setup.bash
```)

== Troubleshooting Common Issues

1. Old ROS processes:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
pkill -9 -f ros
ros2 daemon stop
```)

2. Environment not sourced.
3. Zenoh environment variables not exported.
4. Incorrect Tailscale IP:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
tailscale ip
```)

5. Middleware errors:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
ros2 doctor --report | grep middleware
```)

6. Talker–Listener issues.
7. Error treating timestamp for received data.

First check clock status on both:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
timedatectl status
```)

Look for:
System clock synchronised: yes
NTP service: active

If not synchronised, enable it:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
sudo timedatectl set-ntp true
```)

Then verify again:

#zebraw(
  numbering: false,
  indentation: 2,
  ```bash
timedatectl status
```)

= Version History

#table(
  columns: (auto, auto, auto),
  inset: 10pt,
  align: horizon,
  table.header(
    [*Version*], [*Date*], [*Edited By*],
  ),
  [v2.2], [2026-02-19], [TG],
  [v2.1], [2026-02-19], [TG],
  [v2.0], [2026-02-18], [TG, AO],
  [v1.1], [2026-02-18], [TG],
  [v1.0], [2026-02-17], [TG, AO]
)

