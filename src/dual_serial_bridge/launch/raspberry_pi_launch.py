from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- VOS PORTS USB PERSISTANTS (C'est vital !) ---
    port_mega = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_03536383236351B0F2E1-if00'
    port_uno  = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351C09382-if00'

    return LaunchDescription([
        
        # --- SECTION MEGA (Nouvelle architecture symétrique) ---
        Node(package='dual_serial_bridge', executable='mega_bridge', name='mega_bridge', output='screen', parameters=[{'port': port_mega, 'baudrate': 115200}]),
        Node(package='dual_serial_bridge', executable='mega_driver', name='mega_driver', output='screen'),

        # --- SECTION UNO ---
        Node(package='dual_serial_bridge', executable='uno_bridge', name='uno_bridge', output='screen', parameters=[{'port': port_uno, 'baudrate': 115200}]),
        Node(package='dual_serial_bridge', executable='uno_driver', name='uno_driver', output='screen'),

        # --- SECTION HARDWARE RPI ---
        Node(package='dual_serial_bridge', executable='rpi_relay_node', name='rpi_relay_node', parameters=[{'relay_pin': 26}]),


        # --- SECTION VISION (Caméras optimisées) ---
        Node(
            package='vision_ia',
            executable='camera', 
            name='camera_node',
            output='screen',
            parameters=[
                {'camera_index_1': 0},
                {'topic_name_1': '/video_cam'},
                {'camera_index_2': 2}, 
                {'topic_name_2': '/video_cam_2'}
            ]
        )
    ])