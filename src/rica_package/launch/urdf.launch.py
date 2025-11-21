from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('rica_package')
    urdf_path = os.path.join(share, 'urdf', 'rica.urdf')
    robot_description = open(urdf_path, 'r').read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        # Optionnel si tu veux bouger les joints à la main
        Node(package='joint_state_publisher', executable='joint_state_publisher')
    ])
