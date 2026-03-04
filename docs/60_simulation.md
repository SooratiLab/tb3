# Simulating TurtleBot3

## What is Gazebo?

Gazebo is an open-source robot simulator. Originally, it used the Ignition rendering engine (versions named "Fortress", "Edifice", "Citadel"). The project has since been rebranded back to Gazebo Sim, which now uses Gz (the new rendering engine).

## ROS Topic Integration

**Gazebo Classic (Ignition Citadel/Fortress)**: Integrates directly with ROS 2. Topics like `/cmd_vel`, `/scan`, and `/odom` appear automatically as ROS topics. No remapping is needed.

**Gazebo Sim (Gz)**: Uses its own internal topic naming scheme (e.g., `/world/{name}/model/{model}/link/{link}/sensor/{sensor}/scan`). You must remap these to standard ROS topic names in your launch files:

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    remappings=[('/cmd_vel', '/robot_cmd_vel')]
)
```

The `ros_gz_bridge` package bridges ROS 2 topics with Gazebo Sim topics.

## Overview

Gazebo Sim enables you to test and develop your robot code without physical hardware. This is useful for prototyping, debugging, and CI/CD pipelines.

## Launching a Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This opens a simulated world with a TurtleBot3 robot. You can also create your own simulation environment.

## Using Simulation Time

