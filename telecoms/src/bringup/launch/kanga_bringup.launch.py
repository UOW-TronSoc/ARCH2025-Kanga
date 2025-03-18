from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    # Custom ODrive Node
    odrive_node = Node(
        package='custom_odrive_node',
        executable='custom_odrive_node',  # Make sure this matches your built executable
        name='custom_odrive_node',
        output='screen'
    )

    # Joy Node (Joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',  # Joystick device
            'deadzone': 0.1
        }]
    )

    # Onboard Control Node (Listens to Joystick)
    joy_listener_node = Node(
        package='onboard_control',
        executable='joy_listener',
        name='joy_listener',
        output='screen'
    )

    return LaunchDescription([
        odrive_node,
        joy_node,
        joy_listener_node
    ])
