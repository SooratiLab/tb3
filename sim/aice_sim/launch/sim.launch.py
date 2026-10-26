#!/usr/bin/env python3
"""
Launch a Gazebo Sim Harmonic world with a TurtleBot3 Waffle Pi.

Usage:
    ros2 launch aice_sim sim.launch.py
    ros2 launch aice_sim sim.launch.py world:=base_world
    ros2 launch aice_sim sim.launch.py world:=/path/to/my_world.sdf
    ros2 launch aice_sim sim.launch.py gui:=false

`world` may be a name in this package's worlds/ directory (with or without
the .world/.sdf extension), or an absolute path to any world file.

The robot is included in the arena and house worlds rather than spawned after
startup. On a software-rendered container the world can take a long time to
load, and a spawn request sent too early is regularly lost.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_path(context, world_dir):
    world = LaunchConfiguration('world').perform(context).strip()

    candidates = [world] if os.path.isabs(world) else [
        os.path.join(world_dir, world),
        os.path.join(world_dir, world + '.world'),
        os.path.join(world_dir, world + '.sdf'),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    raise RuntimeError(
        "Cannot find world '{}'. Looked in: {}".format(
            world, ', '.join(os.path.abspath(c) for c in candidates)
        )
    )


def _gazebo(context, world_dir):
    world_path = _world_path(context, world_dir)
    gui = LaunchConfiguration('gui').perform(context).lower() not in (
        'false', '0', 'no', 'off'
    )

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': '-r -s -v2 {}'.format(world_path),
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ]

    if gui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={'gz_args': '-g -v2'}.items(),
            )
        )

    return actions


def generate_launch_description():
    aice_sim_share = get_package_share_directory('aice_sim')
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    world_dir = os.path.join(aice_sim_share, 'worlds')

    resource_path = os.pathsep.join(filter(None, [
        os.path.join(aice_sim_share, 'models'),
        os.path.join(turtlebot3_share, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            'config_file:={}'.format(os.path.join(
                turtlebot3_share, 'params', 'turtlebot3_waffle_pi_bridge.yaml'
            )),
        ],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='arena',
            description='World name in aice_sim/worlds, or an absolute path.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI. Set false for headless use.',
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=_gazebo, args=[world_dir]),
        robot_state_publisher,
        bridge,
        image_bridge,
    ])
