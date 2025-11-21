import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # MODIFICATION ICI :
    pkg_name = 'sysmap_package'
    
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'URDF_v6.SLDASM.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}]
        ),
        Node(
            package=pkg_name, # Utilise la variable sysmap_package
            executable='sysmap_teleop',
            name='sysmap_teleop',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])