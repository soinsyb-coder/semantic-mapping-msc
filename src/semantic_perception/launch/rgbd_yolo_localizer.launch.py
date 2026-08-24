from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('semantic_perception')
    )
    parameter_file = (
        package_share / 'config' / 'rgbd_yolo_localizer.yaml'
    )

    default_model_path = (
        Path.home()
        / 'semantic_ws'
        / 'src'
        / 'semantic_perception'
        / 'models'
        / 'yolo11n.pt'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=str(default_model_path),
            description='Absolute path to the YOLO model weights',
        ),
        Node(
            package='semantic_perception',
            executable='rgbd_yolo_localizer_node',
            name='rgbd_yolo_localizer_node',
            output='screen',
            parameters=[
                str(parameter_file),
                {
                    'model_path': LaunchConfiguration('model_path'),
                },
            ],
        ),
    ])
