# Camera Setup

## Manual Camera Setup

> [!IMPORTANT]  
> If you previously tried to set up the camera, make sure to uninstall all dependencies and remove the `libcamera` repository beforehand.

The instructions to set up the camera manually differs slightly from the ROBOTIS instructions. First install the dependencies (make sure not to install Meson or Ninja from `apt`).

```bash
sudo apt update
sudo apt install -y python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl \
        libtiff-dev pybind11-dev qtbase5-dev libqt5core5a libqt5widgets5 cmake python3-yaml \
        python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install ros-humble-camera-ros
pip install meson ninja
```

Then we can clone the repository and build the package.

```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b v0.5.2 https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build -j 2
ninja -C build install -j 2
```

Then add the following line to you `~/.bashrc` file.

```bash
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

Restart the robot and test the camera.

## Camera Test

On the Turtlebot:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch turtlebot3_bringup camera.launch.py format:=RGB888 width:=640 height:=480
```

Recommended formats:

```bash
BGR888 (64x64 - 3280x2464)
RGB888 (64x64 - 3280x2464)
```

On the Remote PC:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select /camera/image_raw/compressed.

> [!INFO]
> If you receive an error when using the compressed camera topic, install the image transport plugins: 
>
>`sudo apt install ros-humble-image-transport-plugins`



## Image Transport Optimisation

Raw image topics are very high bandwidth (approximately 27 MB/s for 640×480 RGB8 at 30 FPS).

Install plugins:

```bash
sudo apt install ros-humble-image-transport-plugins
```

If you already have a launch file, you can add this node to it:

```python
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
```

Verify:

```bash
ros2 topic list | grep compressed
```

## Useful ROS 2 Commands

```bash
ros2 topic hz <topic_name>
ros2 topic bw <topic_name>
ros2 topic echo <topic_name> | head
ros2 run rqt_image_view rqt_image_view
ros2 run image_tools showimage --ros-args -r image:=<camera_image_topic>
```

Always source after changes:

```bash
source install/setup.bash
```
