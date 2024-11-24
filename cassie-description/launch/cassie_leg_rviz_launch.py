import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share_pth = get_package_share_directory('cassie_description')

    default_rviz_config_path = os.path.join(pkg_share_pth, 'rviz/config.rviz')
    model_file_dir = os.path.join(pkg_share_pth, 'urdf/cassie_rigid_leg.urdf')
    
    # URDF model file
    urdf = open(model_file_dir).read()

    return LaunchDescription([

        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_joint_gui', default_value='True', 
                            description='Provide GUI for joint manipulation'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='/cassie',
            remappings=[('/joint_states', '/cassie/joint_states')],
            condition=IfCondition(LaunchConfiguration('use_joint_gui'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='/cassie',
            remappings=[('/joint_states', '/cassie/joint_states')],
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='rviz_interface_node',
            executable='rviz_interface_node',
            namespace=LaunchConfiguration('namespace'),
            name='rviz_interface_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='/cassie',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        )
    ])
