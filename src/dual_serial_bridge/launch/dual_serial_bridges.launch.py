from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- VOS PORTS USB PERSISTANTS ---
    port_mega = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_7513030393535130D120-if00'
    port_uno = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351C09382-if00'

    return LaunchDescription([
        
        # 1. DRIVER MEGA (Gère le Série directement, PAS de bridge !)
        Node(
            package='dual_serial_bridge',
            executable='mega_driver',
            name='mega_driver',
            output='screen',
            parameters=[{'port': port_mega, 'baudrate': 115200}]
        ),

        # 2. BRIDGE UNO (Gère le Série et publie sur /uno/raw)
        Node(
            package='dual_serial_bridge',
            executable='uno_bridge', # Assurez-vous que c'est le bon nom d'exécutable
            name='uno_bridge',
            output='screen',
            parameters=[{'port': port_uno, 'baudrate': 115200}]
        ),

        # 3. DRIVER UNO (Parse le JSON de /uno/raw vers les topics ROS 2)
        Node(
            package='dual_serial_bridge',
            executable='uno_driver',
            name='uno_driver',
            output='screen'
        )
    ])
