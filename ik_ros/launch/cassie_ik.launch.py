import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import NotSubstitution, AndSubstitution, Command
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(get_package_share_directory('cassie_description'), 'urdf', 'cassie.urdf')
    f = open(urdf_file, 'r')
    robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='ik_ros',
            executable='cassie_ik',
            output='screen',
            emulate_tty=True,
            parameters=[{"robot_description": robot_desc}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('ik_ros'), 
                                           'rviz', 'config.rviz')]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='robot',
            parameters=[{'use_sim_time': True, "robot_description": robot_desc}],
        )
    ])