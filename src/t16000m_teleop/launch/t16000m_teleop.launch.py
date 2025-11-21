from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Driver joystick générique
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',   # adapte si besoin
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,   # répétition des commandes
            }]
        ),

        # Notre nœud de téléop T16000M → /cmd_vel
        Node(
            package='t16000m_teleop',
            executable='t16000m_teleop',
            name='t16000m_teleop',
            output='screen',
            parameters=[{
                'axis_linear': 1,          # Y
                'axis_angular': 0,         # X
                'scale_linear': 0.6,
                'scale_angular': 1.0,
                'enable_button': 0,        # gâchette
                'require_enable_button': True,
                'invert_linear': -1.0,     # pousse le manche → avance
                'invert_angular': 1.0,
            }]
        )
    ])
