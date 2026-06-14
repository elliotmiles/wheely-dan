import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch.actions import LogInfo

from launch.actions import TimerAction

def generate_launch_description():

    package_name = 'wd_bringup'

    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','lidar.launch.py'
                )])
    )

    joystick_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )

    depth_camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'
                )]),
                launch_arguments={
                    'depth_module.profile': '640x480x30',
                    'rgb_camera.profile': '640x480x30',
                    'align_depth.enable': 'true'
                }.items()
    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]),
                launch_arguments={
                    'slam_params_file': 'src/wd_navigation/config/mapper_params_online_async.yaml',
                    'use_sim_time': 'false'
                }.items()
    )

    delayed_slam_toolbox = TimerAction(
    period=3.0,
    actions=[slam_toolbox]
    )
    
    comms = Node(
        package='wd_comms',
        executable='serial_comms',
        output='screen'
    )

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

        rsp_launch,
        lidar_launch,
        joystick_launch,
        comms,
        twist_mux,
        depth_camera,
        delayed_slam_toolbox
    ])