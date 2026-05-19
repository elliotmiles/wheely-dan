import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch.actions import LogInfo

def generate_launch_description():

    package_name = 'wd_bringup'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','lidar.launch.py'
                )])
    )

    joystick_config = os.path.join(get_package_share_directory('wd_control'),
        'config', 'joystick.yaml')
    joystick = Node(
        package='joy_node',
        executable='joy_node',
        output='screen',
        parameters=[joystick_config])

    # twist_mux manages multiple sources of /cmd_vel input
    twist_mux_config = os.path.join(get_package_share_directory('wd_control'),
        'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/cmd_vel')},
        parameters=[
            {'use_sim_time': False},
            twist_mux_config])

    # launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        rsp,
        lidar_launch,
        joystick,
        twist_mux
    ])