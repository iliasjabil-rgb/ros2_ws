import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # --- 1. PONT SÉRIE UNO (Câble USB) ---
        Node(
            package='dual_serial_bridge',
            executable='uno_bridge',
            name='uno_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/arduino_uno',   # Vérifiez vos règles UDEV
                'baudrate': 115200
            }]
        ),

        # --- 2. PONT SÉRIE MEGA (Câble USB) ---
        Node(
            package='dual_serial_bridge',
            executable='mega_bridge',
            name='mega_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/arduino_mega',  # Vérifiez vos règles UDEV
                'baudrate': 115200
            }]
        ),

        # --- 3. DRIVER UNO (LEDs, Capteurs, Watchdog) ---
        Node(
            package='dual_serial_bridge',
            executable='uno_driver',
            name='uno_driver',
            output='screen'
        ),

        # --- 4. DRIVER MEGA (Moteurs, Watchdog) ---
        Node(
            package='dual_serial_bridge',
            executable='mega_driver',
            name='mega_driver',
            output='screen'
        ),

        # --- 5. SYSTEM MONITOR (Voyant RPi) ---
        Node(
            package='dual_serial_bridge',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        ),

    ])
