from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('dual_serial_bridge')
    params = os.path.join(pkg, 'config', 'bridges.params.yaml')

    # MEGA (Bridge + Driver)
    mega_bridge = Node(
        package='dual_serial_bridge',
        executable='mega_bridge',
        name='mega_bridge',
        output='screen',
        parameters=[params])

    mega_driver = Node(
        package='dual_serial_bridge',
        executable='mega_driver',
        name='mega_driver',
        output='screen')

    # UNO (Bridge + Driver)
    uno_bridge = Node(
        package='dual_serial_bridge',
        executable='uno_bridge',
        name='uno_bridge',
        output='screen',
        parameters=[params])
    
    # Note: uno_driver est souvent lancé ici aussi, vérifiez s'il manquait
    uno_driver = Node(
        package='dual_serial_bridge',
        executable='uno_driver',
        name='uno_driver',
        output='screen')

    return LaunchDescription([
        mega_bridge, 
        mega_driver, 
        uno_bridge,
        uno_driver
    ])