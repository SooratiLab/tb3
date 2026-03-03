# Creating a New ROS 2 Package

## Overview

Now that your TurtleBot3 is set up with Zenoh networking over Tailscale, you can begin developing your own custom applications. ROS 2 organises code into packages—self-contained units that contain nodes, libraries, configurations, and other resources. This guide walks you through creating your first package to start controlling and extending your robot's behaviour.

## ROS 2 Workspaces

A ROS 2 **workspace** is a directory containing your source code packages. The standard layout has a `src` folder for source code, which gets built into `install`, `build`, and `log` directories:

```
ros2_ws/
├── src/          # Your source packages
├── build/        # Build artifacts
├── install/      # Generated setup files
└── log/          # Build logs
```

To use a workspace, you must source its `setup.bash` file:

```bash
source ~/ros2_ws/install/setup.bash
```

This adds the packages in that workspace to your ROS 2 environment. Create a workspace if you haven't already:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

> [!IMPORTANT]
> Do not add your own packages to the TurtleBot3 workspace (`turtlebot3_ws`). Keep your packages in a separate workspace to avoid conflicts when updating the TurtleBot3 packages.

## Where to Run Nodes

A typical setup for TurtleBot3 development is to run your custom application code on your laptop (the remote PC), while only running the robot bringup on the robot itself. This keeps the robot lightweight and allows you to develop and test code without frequently accessing the robot.

The `turtlebot3_bringup` package on the robot handles:
- Launching the robot's hardware drivers
- Publishing sensor data (camera, LiDAR, IMU)
- Subscribing to `cmd_vel` for motor control

Your custom nodes on the laptop can then:
- Subscribe to sensor topics from the robot
- Process data and make decisions
- Publish `cmd_vel` commands back to the robot

Both the laptop and robot must be on the same ROS domain and connected via Zenoh for this to work.

## Topics, Messages, and QoS

### Topics

Topics are named buses where nodes exchange messages. A node can **publish** messages to a topic or **subscribe** to receive messages from a topic. Many nodes can publish to and subscribe from the same topic.

### Common TurtleBot3 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear/angular) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/odom` | `nav_msgs/Odometry` | Odometry (position/velocity) |

### Quality of Service (QoS)

QoS controls how messages are delivered. Key policies include:

- **Reliability**: `RELIABLE` ensures messages are delivered (like TCP), while `BEST_EFFORT` drops messages if they're late (like UDP).
- **History**: `KEEP_LAST(n)` keeps the last n messages, `KEEP_ALL` keeps all.

For sensors (LiDAR, camera), use `BEST_EFFORT` since old data is less useful. For commands (`cmd_vel`), use `RELIABLE` to ensure commands arrive.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
self.sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
```

## Creating a Python Package with rclpy

> [!NOTE]
> If you would prefer to use C++, we recommend the [`rclcpp` documentation](https://docs.ros.org/en/rolling/p/rclcpp/). For Rust, see the [`r2r` documentation](https://docs.rs/r2r/latest/r2r/).

This guide uses rclpy, the Python client library for ROS 2.

### Prerequisites

Ensure you have ROS 2 Humble installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

### Creating the Package

1. Navigate to your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
```

2. Create the package using the ROS 2 tool:

```bash
ros2 pkg create --build-type ament_python my_robot_package
```

This creates a basic package structure:

```
my_robot_package/
├── package.xml
├── my_robot_package/
│   └── __init__.py
└── setup.py
```

### Writing Your First Node

1. Create a simple Python node in `my_robot_package/my_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('My node has started')

    def main(args=None):
        rclpy.init(args=args)
        node = MyNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. Make the file executable:

```bash
chmod +x my_robot_package/my_node.py
```

3. Update `setup.py` to include the executable:

```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=['share/ament_index/resource_index/packages'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='My first TurtleBot3 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
        ],
    },
)
```

### Building and Running

1. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

2. Source the workspace:

```bash
source install/setup.bash
```

3. Run the node:

```bash
ros2 run my_robot_package my_node
```

## Understanding colcon

`colcon` is the command-line tool for building and testing ROS 2 packages. It stands for "collective construction" and is the successor to `catkin_make` from ROS 1.

### Common colcon Commands

- **Build all packages** in your workspace:

```bash
colcon build
```

- **Build a specific package**:

```bash
colcon build --packages-select my_robot_package
```

- **Build with symbols for debugging**:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

- **Allow testing**:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

- **Run tests**:

```bash
colcon test
```

- **Clean the build**:

```bash
rm -rf build/ install/ log/
```

- **List packages**:

```bash
colcon list
```

## Launch Files

Launch files allow you to start multiple nodes and configure their parameters simultaneously. They are essential for bringing up complex robot systems.

### Creating a Launch File

1. Create a `launch` directory in your package:

```bash
mkdir -p my_robot_package/launch
```

2. Create `my_robot_package/launch/my_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_node',
            name='my_renamed_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
```

### Calling Other Launch Files

You can include other launch files, such as the TurtleBot3 robot bringup:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot3_bringup = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch/robot.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_bringup),
        ),
        Node(
            package='my_robot_package',
            executable='my_node',
            name='my_node',
            output='screen',
        ),
    ])
```

### Making Launch Files Discoverable

Update `setup.py` to include the launch files:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob(package_name + '/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='My first TurtleBot3 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
        ],
    },
)
```

You also need to add `ament_index_python` to your dependencies in `package.xml`:

```xml
<exec_depend>ament_index_python</exec_depend>
```

### Running a Launch File

```bash
ros2 launch my_robot_package my_launch.py
```

## ROS 2 Messages

Messages are the way nodes communicate with each other. They define the data structure for topics, services, and actions. ROS 2 comes with many built-in message types, such as `std_msgs/String`, `geometry_msgs/Twist`, and `sensor_msgs/Image`.

### Using Existing Messages

To use a message in your node, import it and specify the type when creating a publisher or subscriber:

```python
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.timer = self.create_timer(1.0, self.publish_hello)
    
    def publish_hello(self):
        msg = String()
        msg.data = 'Hello ROS 2!'
        self.pub.publish(msg)
    
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Linear x: {msg.linear.x}, Angular z: {msg.angular.z}')
```

### Creating Custom Messages

When the standard messages don't fit your needs, you can create custom messages.

1. Create a separate package for your message:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_msgs
```

2. Create the message definition in `my_robot_msgs/msg/CustomMessage.msg`:

```
bool is_active
string name
float64[] sensor_readings
geometry_msgs/Pose pose
```

3. Update `package.xml` to declare the message:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4. Update `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_msgs)

if(CMAKE_COMPILER_VERSION VERSION_LESS 8)
  find_package(ament_cmake REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

5. Build the message package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_msgs
source install/setup.bash
```

6. Use the custom message in your Python node:

```python
from my_robot_msgs.msg import CustomMessage

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher(CustomMessage, 'custom_topic', 10)
    
    def publish_custom(self):
        msg = CustomMessage()
        msg.is_active = True
        msg.name = 'my_robot'
        msg.sensor_readings = [1.0, 2.0, 3.0]
        self.pub.publish(msg)
```

> [!NOTE]
> If using C++, add the dependency to your `package.xml` and `CMakeLists.txt`, then rebuild. Python auto-generates the bindings when the message package is built.
