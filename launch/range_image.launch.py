import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription

def generate_launch_description():
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{message}"
    os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
    # Path to YAML config file for parameters
    param_config_dir = os.path.join(
        get_package_share_directory('lidar_slam_utils'),
        'config',
        'conf_image_mid360.yaml'
    )
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='lidar_slam_utils',
            executable='range_image_node',
            output='screen',
            parameters=[param_config_dir]  # Load parameters from the YAML file
        ),
        launch_ros.actions.Node(
            package='rqt_image_view',
            namespace='',
            executable='rqt_image_view',
            name='rqt_image_view',
        ),
    ])
