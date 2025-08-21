import os

import launch
import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{message}"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

    return LaunchDescription([
        launch_ros.actions.Node(
            package='lidar_slam_utils',
            executable='save_cloud_node',
            name='save_cloud_node',
            output='screen',
            parameters=[
                {"topic.in": "/livox/lidar"},
                {"format.in": 0},  # 0: CustomMsg, 1: PointCloud2
                {"format.out": "pcd"},
                {"path": "./clouds/"},
            ]
        )
    ])