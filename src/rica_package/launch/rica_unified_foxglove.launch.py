from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ip            = LaunchConfiguration('ip')
    port          = LaunchConfiguration('port')
    xbox_yaml     = LaunchConfiguration('xbox_yaml')
    fox_port      = LaunchConfiguration('foxglove_port')

    # unified teleop params (joystick GAUCHE → pattes AR ; joystick DROIT → base via teleop_twist_joy)
    btn_rb        = LaunchConfiguration('btn_rb')
    ax_lt         = LaunchConfiguration('ax_lt')
    ax_rt         = LaunchConfiguration('ax_rt')
    trig_thr      = LaunchConfiguration('trig_thr')
    ax_leg        = LaunchConfiguration('ax_leg')
    deadzone_leg  = LaunchConfiguration('deadzone_leg')
    inv_leg       = LaunchConfiguration('inv_leg')
    max_speed     = LaunchConfiguration('max_speed_mps')

    return LaunchDescription([
        # ---- Args ----
        DeclareLaunchArgument('ip',              default_value='192.168.0.2'),
        DeclareLaunchArgument('port',            default_value='2009'),
        DeclareLaunchArgument('foxglove_port',   default_value='8766'),
        DeclareLaunchArgument('xbox_yaml', default_value=PathJoinSubstitution([
            FindPackageShare('rica_package'), 'config', 'xbox.yaml'
        ])),
        # manette / pattes
        DeclareLaunchArgument('btn_rb',        default_value='5'),
        DeclareLaunchArgument('ax_lt',         default_value='2'),
        DeclareLaunchArgument('ax_rt',         default_value='5'),
        DeclareLaunchArgument('trig_thr',      default_value='0.5'),
        DeclareLaunchArgument('ax_leg',        default_value='1'),    # stick G vertical
        DeclareLaunchArgument('deadzone_leg',  default_value='0.10'),
        DeclareLaunchArgument('inv_leg',       default_value='false'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.6'),

        # ---- Nodes ----
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),

        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_node',
             parameters=[xbox_yaml], output='screen'),

        # Script unifié (une seule socket TCP → pas de rica_bridge ici)
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/ilias/ros2_ws/src/rica_package/scripts/rica_unified_teleop.py',
                '--ros-args',
                '-p', ['ip:=', ip],
                '-p', ['port:=', port],
                '-p', ['max_speed_mps:=', max_speed],
                '-p', ['btn_rb:=', btn_rb],
                '-p', ['ax_lt:=', ax_lt],
                '-p', ['ax_rt:=', ax_rt],
                '-p', ['trig_thr:=', trig_thr],
                '-p', ['ax_leg:=', ax_leg],
                '-p', ['deadzone_leg:=', deadzone_leg],
                '-p', ['inv_leg:=', inv_leg],
            ],
            shell=False
        ),

        # Foxglove bridge (WebSocket)
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge', output='screen',
             parameters=[{'port': fox_port, 'address': '0.0.0.0'}]),
    ])
