from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():

    robot_description = Command([
        'xacro ',
        os.path.join(
            os.getenv('HOME'),
            'logicbomb/Projects/SLAM/wheely-dan/src/wd_description/src/robot.urdf.xacro'
        )
    ])

    return LaunchDescription([

        # Start Gazebo Sim
        ExecuteProcess(
            cmd=['gz', 'sim', '-r'],
            output='screen'
        ),

        # Publish robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'wheely_dan'
            ],
            output='screen'
        ),

        # Bridge topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
            ]
        ),
    ])