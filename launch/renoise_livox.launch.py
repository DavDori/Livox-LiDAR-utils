import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{message}"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

    # Path to YAML config file for parameters
    rviz_config_dir = os.path.join(
        get_package_share_directory('lidar_slam_utils'),
        'rviz',
        'rviz_noisy.rviz'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )

    rviz_use = LaunchConfiguration('rviz')

    noise_node = launch_ros.actions.Node(
        package='lidar_slam_utils',
        executable='livox_renoise_node',
        name='livox_renoise_node',
        output='screen',
        parameters=[
            {"topic.in": "/livox/lidar"},
            {"topic.out": "/livox/lidar_noisy"},
            {"sigma.range_m": 0.1},
            {"sigma.angle_deg": 1.0},
        ]
    )

    conv_node = launch_ros.actions.Node(
        package='lidar_slam_utils',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        output='screen',
        parameters=[
            {"topic.in": "/livox/lidar_noisy"},
            {"topic.out": "/livox/cloud_noisy"},
            {"max_range_m": 40.0},
            {"enable_lightweight": True},
        ],
        condition=IfCondition(rviz_use)
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(rviz_use)
    )
    return LaunchDescription([
        declare_rviz_arg,
        noise_node,
        conv_node,
        rviz_node,
    ])