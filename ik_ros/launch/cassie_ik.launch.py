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

    urdf = xacro.process_file(os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro'), 
                              mappings={'name': "ur", 'ur_type' : 'ur5'})
    robot_desc = urdf.toprettyxml(indent='  ')

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
            executable='rviz2'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='robot',
            parameters=[{'use_sim_time': True, "robot_description": robot_desc}],
        )
    ])