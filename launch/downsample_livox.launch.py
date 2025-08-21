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

    rviz_config_dir = os.path.join(
        get_package_share_directory('lidar_slam_utils'),
        'rviz',
        'rviz_ds.rviz'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_ds_arg = DeclareLaunchArgument(
        'd', default_value='3',
        description='Decimation factor for downsampling the LiDAR data'
    )
    rviz_use = LaunchConfiguration('rviz')
    decimation = LaunchConfiguration('d')

    ds_node = launch_ros.actions.Node(
        package='lidar_slam_utils',
        executable='livox_downsample_node',
        name='livox_downsample_node',
        output='screen',
        parameters=[
            {"topic.in": "/livox/lidar"},
            {"topic.out": "/livox/lidar_ds"},
            {"decimation": decimation},
        ]
    )

    conv_node = launch_ros.actions.Node(
        package='lidar_slam_utils',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        output='screen',
        parameters=[
            {"topic.in": "/livox/lidar"},
            {"topic.out": "/livox/cloud"},
            {"max_range_m": 40.0},
            {"enable_lightweight": True},
        ],
        condition=IfCondition(rviz_use)
    )

    conv_ds_node = launch_ros.actions.Node(
        package='lidar_slam_utils',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        output='screen',
        parameters=[
            {"topic.in": "/livox/lidar_ds"},
            {"topic.out": "/livox/cloud_ds"},
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
        declare_ds_arg,
        ds_node,
        conv_node,
        conv_ds_node,
        rviz_node,
    ])