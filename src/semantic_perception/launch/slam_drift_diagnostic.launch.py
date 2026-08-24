from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the read-only SLAM TF drift diagnostic."""
    package_share = Path(
        get_package_share_directory('semantic_perception')
    )
    parameter_file = (
        package_share / 'config' / 'slam_drift_diagnostic.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_path',
            default_value='',
            description='Optional output CSV path',
        ),
        Node(
            package='semantic_perception',
            executable='slam_drift_diagnostic_node',
            name='slam_drift_diagnostic_node',
            output='screen',
            parameters=[
                str(parameter_file),
                {'csv_path': LaunchConfiguration('csv_path')},
            ],
        ),
    ])
