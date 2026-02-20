from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- Configurations de Lancement (Variables) ----
    ip            = LaunchConfiguration('ip')
    port          = LaunchConfiguration('port')
    xbox_yaml     = LaunchConfiguration('xbox_yaml')
    fox_port      = LaunchConfiguration('foxglove_port')

    btn_rb        = LaunchConfiguration('btn_rb')
    ax_lt         = LaunchConfiguration('ax_lt')
    ax_rt         = LaunchConfiguration('ax_rt')
    trig_thr      = LaunchConfiguration('trig_thr')
    ax_leg        = LaunchConfiguration('ax_leg')
    deadzone_leg  = LaunchConfiguration('deadzone_leg')
    inv_leg       = LaunchConfiguration('inv_leg')
    max_speed     = LaunchConfiguration('max_speed_mps')
    poll_hz       = LaunchConfiguration('poll_hz') # <-- AJOUT ICI

    return LaunchDescription([
        # ---- Arguments par défaut ----
        DeclareLaunchArgument('ip',              default_value='192.168.0.2'),
        DeclareLaunchArgument('port',            default_value='2009'),
        DeclareLaunchArgument('foxglove_port',   default_value='8766'),
        DeclareLaunchArgument('xbox_yaml', default_value=PathJoinSubstitution([
            FindPackageShare('rica_package'), 'config', 'xbox.yaml'
        ])),
        DeclareLaunchArgument('btn_rb',        default_value='5'),
        DeclareLaunchArgument('ax_lt',         default_value='2'),
        DeclareLaunchArgument('ax_rt',         default_value='5'),
        DeclareLaunchArgument('trig_thr',      default_value='0.5'),
        DeclareLaunchArgument('ax_leg',        default_value='1'),    
        DeclareLaunchArgument('deadzone_leg',  default_value='0.10'),
        DeclareLaunchArgument('inv_leg',       default_value='false'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.6'),
        DeclareLaunchArgument('poll_hz',       default_value='1.0'), # <-- AJOUT ICI

        # ---- Nœuds (Nodes) ----
        
        # Manette
        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node', 
            output='screen'
        ),

        # Traduction Manette -> cmd_vel
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node', 
            name='teleop_node',
            parameters=[xbox_yaml], 
            output='screen'
        ),

        # Script unifié
        Node(
            package='rica_package',
            executable='rica_unified_teleop',
            name='rica_unified_teleop',
            output='screen',
            parameters=[{
                'ip': ip,
                'port': port,
                'max_speed_mps': max_speed,
                'btn_rb': btn_rb,
                'ax_lt': ax_lt,
                'ax_rt': ax_rt,
                'trig_thr': trig_thr,
                'ax_leg': ax_leg,
                'deadzone_leg': deadzone_leg,
                'inv_leg': inv_leg,
                'poll_hz': poll_hz, # <-- AJOUT ICI
            }]
        ),

        # Foxglove bridge (WebSocket)
        Node(
            package='foxglove_bridge', 
            executable='foxglove_bridge',
            name='foxglove_bridge', 
            output='screen',
            parameters=[{
                'port': fox_port, 
                'address': '0.0.0.0'
            }]
        ),
    ])