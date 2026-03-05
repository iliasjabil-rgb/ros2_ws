import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. CONFIGURATION URDF (Robot State Publisher) ---
    pkg_sysmap = 'sysmap_package'
    urdf_file = os.path.join(get_package_share_directory(pkg_sysmap), 'urdf', 'URDF_v6.SLDASM.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    

    # --- 2. CONFIGURATION XBOX / RICA ---
    xbox_yaml = PathJoinSubstitution([FindPackageShare('rica_package'), 'config', 'xbox.yaml'])

    return LaunchDescription([
        # --- INFRASTRUCTURE IHM ---
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen', parameters=[{'port': 8765, 'address': '0.0.0.0'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot_desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui'),
        Node(package='sysmap_package', executable='csv_recorder', name='csv_recorder', output='screen'),
        Node(package='sysmap_package', executable='recorder_manager', name='recorder_manager', output='screen'),

        # --- WATCHDOG DE SYSTÈME (MONITOR) ---
        Node(package='dual_serial_bridge', executable='system_monitor', name='system_monitor'),
        
        # --- VISION IA ---
        Node(package='vision_ia', executable='vision_IA', name='vision_IA', output='screen'),

        # --- MANETTE 1 : XBOX (Base RICA) ---
        Node(
            package='joy_linux',          # <--- CHANGÉ ICI
            executable='joy_linux_node',  # <--- CHANGÉ ICI
            name='joy_xbox', 
            parameters=[{
                'dev': '/dev/input/by-id/usb-©Microsoft_Corporation_Controller_17ACDA2-joystick', 
                'deadzone': 0.05
            }], 
            remappings=[('/joy', '/xbox/joy')]
        ),

        # --- MANETTE 2 : THRUSTMASTER (Bras SRS) ---
        Node(
            package='joy_linux',          # <--- CHANGÉ ICI
            executable='joy_linux_node',  # <--- CHANGÉ ICI
            name='joy_thrustmaster', 
            parameters=[{
                'dev': '/dev/input/by-id/usb-Thrustmaster_T.16000M-joystick', 
                'deadzone': 0.05
            }], 
            remappings=[('/joy', '/thrustmaster/joy')]
        ),

        # --- NŒUD DE CONTRÔLE DU BRAS (SYSMAP) ---
        Node(
            package='sysmap_package', 
            executable='sysmap_joystick', 
            name='sysmap_joystick', 
            output='screen',
            remappings=[('/joy', '/thrustmaster/joy')] 
        ),
        # Traduction Xbox -> cmd_vel
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_node', parameters=[xbox_yaml], remappings=[('/joy', '/xbox/joy')]),

        # --- COMMUNICATIONS TCP RICA ---
        Node(
            package='rica_package', # <-- Corrigé ici
            executable='rica_unified_teleop',
            name='rica_unified_teleop',
            output='screen',
            parameters=[{
                'ip': '192.168.0.2',
                'port': 2009,
                'max_speed_mps': 0.6,
                'btn_rb': 5,
                'ax_lt': 2,
                'ax_rt': 5,
                'trig_thr': 0.5,
                'ax_leg': 1,
                'deadzone_leg': 0.10,
                'inv_leg': False,
                'poll_hz': 1.0,
            }]
        ),

        # --- BATTERIE SIMULÉE ---
        Node(package='sysmap_package', executable='battery_simulator', name='battery_simulator', output='screen'),

        # --- ANIMATION SRS ---
        Node(package='sysmap_package', executable='srs_led_node', name='srs_led_node', output='screen'),

        # --- INTELLIGENCE ---
        Node(package='sysmap_package', executable='robot_brain', name='robot_brain', output='screen'),


        # Le Filtre de Madgwick qui calcule l'orientation 3D
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',  
            name='imu_filter',
            parameters=[{'use_mag': False, 'publish_tf': False}],
            remappings=[
                ('/imu/data_raw', '/uno/imu'),      # Il écoute votre capteur
                ('/imu/data', '/uno/imu_filtered')  # Il crée un nouveau topic avec l'Orientation !
            ]
        ),
    ])