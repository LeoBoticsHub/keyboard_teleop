from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context):

    config_name = LaunchConfiguration("config_name").perform(context)
    
    config = os.path.join(
        get_package_share_directory('keyboard_teleop'),
        'config',
        'ros2',
        config_name + '.yaml'
    )

    keyboard_node = Node(
        package='keyboard_teleop',
        executable='ros2_keyboard_teleop.py',
        name='keyboard_teleop',
        emulate_tty=True,
        parameters = [config],
        output='screen'
    )

    return [keyboard_node]


def generate_launch_description():

    config_arg = DeclareLaunchArgument('config_name', default_value='keyboard_teleop')

    return LaunchDescription([config_arg] + [OpaqueFunction(function=launch_setup)])