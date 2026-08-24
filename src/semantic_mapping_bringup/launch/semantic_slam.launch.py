#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    home = os.path.expanduser('~')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # Absolute paths
    # ------------------------------------------------------------------

    world_file = os.path.join(
        home,
        'semantic_ws',
        'src',
        'aws-robomaker-small-house-world',
        'worlds',
        'small_house.world',
    )

    robot_urdf = os.path.join(
        home,
        'semantic_ws',
        'src',
        'turtlebot3',
        'turtlebot3_description',
        'urdf',
        'harmonic_waffle_rgbd.urdf',
    )

    aws_model_path = os.path.join(
        home,
        'semantic_ws',
        'src',
        'aws-robomaker-small-house-world',
        'models',
    )

    # package://turtlebot3_description/... needs the parent directory
    # containing turtlebot3_description.
    turtlebot3_resource_path = os.path.join(
        home,
        'semantic_ws',
        'src',
        'turtlebot3',
    )

    ros_share_path = '/opt/ros/jazzy/share'

    existing_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        '',
    )

    resource_paths = [
        aws_model_path,
        turtlebot3_resource_path,
        ros_share_path,
    ]

    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    gz_resource_path = os.pathsep.join(resource_paths)

    if not os.path.isfile(world_file):
        raise FileNotFoundError(
            f'Gazebo world not found: {world_file}'
        )

    if not os.path.isfile(robot_urdf):
        raise FileNotFoundError(
            f'Robot URDF not found: {robot_urdf}'
        )

    with open(robot_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # ------------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    # ------------------------------------------------------------------
    # Robot description / TF
    # ------------------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Spawn robot explicitly into world "default"
    # ------------------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3_waffle',
        output='screen',
        arguments=[
            '-world', 'default',
            '-name', 'turtlebot3_waffle',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25',
        ],
    )

    # ------------------------------------------------------------------
    # Gazebo ↔ ROS 2 bridge
    # ------------------------------------------------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # ROS 2 -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Gazebo -> ROS 2
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # RGB-D point cloud
    # ------------------------------------------------------------------

    point_cloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='rgbd_point_cloud',
        output='screen',
        remappings=[
            (
                'depth_registered/image_rect',
                '/camera/depth_image',
            ),
            (
                'rgb/image_rect_color',
                '/camera/image',
            ),
            (
                'rgb/camera_info',
                '/camera/camera_info',
            ),
            (
                'points',
                '/camera/points',
            ),
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'exact_sync': False,
                'queue_size': 30,
            }
        ],
    )

    # ------------------------------------------------------------------
    # RTAB-Map
    # ------------------------------------------------------------------

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            )
        ),
        launch_arguments={
            'rtabmap_args': (
            '--delete_db_on_start '
                '--RGBD/LinearUpdate 0.05 '
                '--RGBD/AngularUpdate 0.05'
            ),
            'rgb_topic': '/camera/image',
            'depth_topic': '/camera/depth_image',
            'camera_info_topic': '/camera/camera_info',
            'odom_topic': '/odom',
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'visual_odometry': 'false',
            'approx_sync': 'true',
            'rgbd_sync': 'true',
            'approx_rgbd_sync': 'true',
            'qos': '2',
            'use_sim_time': 'true',
            'rtabmap_viz': 'false',
            'rviz': 'false',
        }.items(),
    )

    # ------------------------------------------------------------------
    # Relay RTAB-Map OccupancyGrid to /map
    # ------------------------------------------------------------------

    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='rtabmap_map_relay',
        output='screen',
        arguments=[
            '/rtabmap/map',
            '/map',
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------

    rtabmap_rviz_config = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'config',
        'rgbd.rviz',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='semantic_mapping_rviz',
        output='screen',
        arguments=[
            '-d',
            rtabmap_rviz_config,
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Startup sequence
    # ------------------------------------------------------------------

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time',
        ),

        # Make Gazebo able to locate the house and TurtleBot mesh files.
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path,
        ),

        # Retain an additional file-search path for mesh resources.
        SetEnvironmentVariable(
            name='GZ_FILE_PATH',
            value=gz_resource_path,
        ),

        gazebo,
        robot_state_publisher,

        # Bridge can wait for Gazebo topics to appear.
        TimerAction(
            period=8.0,
            actions=[bridge],
        ),

        # The AWS world is heavy, so allow more time before spawning.
        TimerAction(
            period=15.0,
            actions=[spawn_robot],
        ),

        # Wait for the spawned sensors and bridge.
        TimerAction(
            period=15.0,
            actions=[point_cloud],
        ),

        # Start RTAB-Map only after odom and RGB-D data are expected.
        TimerAction(
            period=28.0,
            actions=[rtabmap],
        ),

        # Relay the generated map.
        TimerAction(
            period=23.0,
            actions=[map_relay],
        ),

        # Open RViz after the TF tree and mapping nodes initialise.
        TimerAction(
            period=35.0,
            actions=[rviz],
        ),
    ])
