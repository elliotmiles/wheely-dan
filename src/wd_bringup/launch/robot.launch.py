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

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': use_ros2_control}.items()
    )

    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','lidar.launch.py'
                )])
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            '--controller-ros-args',
            '-r /diff_cont/cmd_vel:=/cmd_vel'
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager-timeout", 
            "50"
        ],
    )

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

    # Launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        LogInfo(msg=["ros2_control enabled: ", use_ros2_control]),

        rsp,
        lidar_launch,
        diff_drive_spawner,
        joint_broad_spawner,
        twist_mux
    ])