#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'semantic_mapping_bringup'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    explore_delay = LaunchConfiguration('explore_delay')

    nav2_launch_file = os.path.join(
        package_share,
        'launch',
        'nav2_manual.launch.py',
    )

    explore_params_file = os.path.join(
        package_share,
        'config',
        'explore_rtabmap.yaml',
    )

    if not os.path.isfile(nav2_launch_file):
        raise FileNotFoundError(
            f'Nav2 launch file not found: {nav2_launch_file}'
        )

    if not os.path.isfile(explore_params_file):
        raise FileNotFoundError(
            f'Explore parameter file not found: '
            f'{explore_params_file}'
        )

    # Start the already validated Nav2 launch.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_launch_file
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    # Start frontier exploration after Nav2 has had time to configure
    # and activate its lifecycle nodes.
    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params_file,
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes',
        ),

        DeclareLaunchArgument(
            'explore_delay',
            default_value='12.0',
            description=(
                'Delay in seconds before starting explore_lite'
            ),
        ),

        nav2,

        TimerAction(
            period=explore_delay,
            actions=[
                explore_lite,
            ],
        ),
    ])
