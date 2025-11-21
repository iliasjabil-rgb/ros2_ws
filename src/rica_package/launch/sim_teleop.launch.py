from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    share = get_package_share_directory('rica_package')
    params_yaml = os.path.join(share, 'config', 'xbox.yaml')
    urdf_path  = os.path.join(share, 'urdf', 'rica.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    return LaunchDescription([
        Node(
        package='joy', executable='joy_node', name='joy',
        parameters=[{'deadzone': 0.15, 'autorepeat_rate': 0.0}],  # << deadzone 
        ),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[params_yaml], output='screen'),
        Node(package='rica_package', executable='diff_drive_sim', name='diff_drive_sim',
             parameters=[{'wheel_separation':0.30,'wheel_radius':0.05,'rate_hz':50.0}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge'),
        Node(package='rica_package', executable='joint_teleop', name='joint_teleop', output='screen',parameters= [{
               'btn_deadman_rb': 5,
               'btn_lt': -1, 'btn_rt': -1,        # on n’utilise PAS les boutons, mais les axes
               'lt_axis_index': 2, 'lt_axis_active_when': 'low',
               'rt_axis_index': 5, 'rt_axis_active_when': 'low',
               'trigger_threshold': 0.9,         # au repos ~ -1.0 -> pas actif ; appuyé > +0.9 -> actif
               'axis_left_y': 1,
               'rate_hz': 50.0, 'deadzone': 0.12, 'rate_rad_per_s': 1.2,
               }]
               ),


    ])
