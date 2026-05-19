import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # process the URDF file
    pkg_path = os.path.join(get_package_share_directory('wd_description'))
    xacro_file = os.path.join(pkg_path,'src','robot.urdf.xacro')
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # create a robot_state_publisher node
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    return LaunchDescription([
        node_robot_state_publisher
    ])
