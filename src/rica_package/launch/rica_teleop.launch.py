from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('rica_package')
    params_yaml = os.path.join(share, 'config', 'xbox.yaml')
    urdf_path  = os.path.join(share, 'urdf', 'rica.urdf')
    robot_description = open(urdf_path, 'r').read()

    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy'),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[params_yaml]),

        Node(package='rica_package', executable='rica_bridge', name='rica_bridge',
             parameters=[{
                 'ip':'192.168.0.2', 'port':2009,
                 'wheel_separation':0.30, 'wheel_radius':0.05, 'ticks_per_rev':2048
             }]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge')
    ])
