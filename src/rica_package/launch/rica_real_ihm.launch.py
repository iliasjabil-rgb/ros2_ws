from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ip   = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')
    fox  = LaunchConfiguration('foxglove_port')

    enable_button   = LaunchConfiguration('enable_button')
    enable_turbo    = LaunchConfiguration('enable_turbo_button')
    axis_linear_x   = LaunchConfiguration('axis_linear_x')
    axis_yaw        = LaunchConfiguration('axis_yaw')
    scale_lin       = LaunchConfiguration('scale_linear_x')
    scale_lin_turbo = LaunchConfiguration('scale_linear_turbo_x')
    scale_yaw       = LaunchConfiguration('scale_angular_yaw')
    scale_yaw_turbo = LaunchConfiguration('scale_angular_turbo_yaw')
    deadzone        = LaunchConfiguration('deadzone')

    btn_rb   = LaunchConfiguration('btn_rb')
    btn_lt   = LaunchConfiguration('btn_lt')
    btn_rt   = LaunchConfiguration('btn_rt')
    ax_lt    = LaunchConfiguration('ax_lt')
    ax_rt    = LaunchConfiguration('ax_rt')
    trig_thr = LaunchConfiguration('trig_thr')
    ax_leg   = LaunchConfiguration('ax_leg')
    inv_leg  = LaunchConfiguration('inv_leg')
    max_speed= LaunchConfiguration('max_speed_mps')
    poll_hz  = LaunchConfiguration('poll_hz')

    script_path = PathJoinSubstitution([FindPackageShare('rica_package'), 'scripts', 'rica_unified_teleop.py'])
    xbox_yaml   = PathJoinSubstitution([FindPackageShare('rica_package'), 'config', 'xbox.yaml'])

    return LaunchDescription([
        # Args
        DeclareLaunchArgument('ip',            default_value='192.168.0.2'),
        DeclareLaunchArgument('port',          default_value='2009'),
        DeclareLaunchArgument('foxglove_port', default_value='8770'),

        DeclareLaunchArgument('enable_button',        default_value='5'),
        DeclareLaunchArgument('enable_turbo_button',  default_value='7'),
        DeclareLaunchArgument('axis_linear_x',        default_value='5'),
        DeclareLaunchArgument('axis_yaw',             default_value='2'),
        DeclareLaunchArgument('scale_linear_x',       default_value='0.6'),
        DeclareLaunchArgument('scale_linear_turbo_x', default_value='1.0'),
        DeclareLaunchArgument('scale_angular_yaw',    default_value='0.8'),
        DeclareLaunchArgument('scale_angular_turbo_yaw', default_value='1.2'),
        DeclareLaunchArgument('deadzone',             default_value='0.05'),

        DeclareLaunchArgument('btn_rb',        default_value='5'),
        DeclareLaunchArgument('btn_lt',        default_value='-1'),
        DeclareLaunchArgument('btn_rt',        default_value='-1'),
        DeclareLaunchArgument('ax_lt',         default_value='2'),
        DeclareLaunchArgument('ax_rt',         default_value='5'),
        DeclareLaunchArgument('trig_thr',      default_value='0.5'),
        DeclareLaunchArgument('ax_leg',        default_value='1'),
        DeclareLaunchArgument('inv_leg',       default_value='false'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.6'),
        DeclareLaunchArgument('poll_hz',       default_value='4.0'),

        # Manette + /cmd_vel (stick droit)
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             parameters=[xbox_yaml], output='screen'),

        # Script unifié : 1 seule socket → roues+pattes+IHM
        ExecuteProcess(
            cmd=[
                'python3', script_path, '--ros-args',
                '-p', ['ip:=', ip], '-p', ['port:=', port],
                '-p', ['max_speed_mps:=', max_speed],
                '-p', ['btn_rb:=', btn_rb], '-p', ['btn_lt:=', btn_lt], '-p', ['btn_rt:=', btn_rt],
                '-p', ['ax_lt:=', ax_lt], '-p', ['ax_rt:=', ax_rt], '-p', ['trig_thr:=', trig_thr],
                '-p', ['ax_leg:=', ax_leg], '-p', ['inv_leg:=', inv_leg], '-p', ['poll_hz:=', poll_hz],
            ],
            shell=False
        ),

        # Foxglove
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
             parameters=[{'address': '0.0.0.0', 'port': fox}]),
    ])
