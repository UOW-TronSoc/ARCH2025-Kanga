import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # Launch the ODrive node
        Node(
            package='custom_odrive_node',
            executable='custom_odrive_node',
            name='custom_odrive_node',
            output='screen'
        ),

        # Launch the multi-camera node
        Node(
            package='multi_camera_node',
            executable='multi_camera_node',
            name='multi_camera_node',
            output='screen'
        ),

        # Launch science feedback node
        Node(
            package='science_control',
            executable='science_feedback',
            name='science_feedback',
            output='screen'
        ),

        # Launch science control node
        Node(
            package='science_control',
            executable='science_control',
            name='science_control',
            output='screen'          
        ),

        # Include the RealSense D435i launch file from rtabmap_examples
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('kanga_launch'), 'launch', 'realsense_d435i_color_launch.py')
            ])
        ),
    ])
