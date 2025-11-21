from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('dual_serial_bridge')
    params = os.path.join(pkg, 'config', 'bridges.params.yaml')

    mega = Node(
        package='dual_serial_bridge',
        executable='mega_bridge',
        name='mega_bridge',
        output='screen',
        parameters=[params])

    uno = Node(
        package='dual_serial_bridge',
        executable='uno_bridge',
        name='uno_bridge',
        output='screen',
        parameters=[params])

    return LaunchDescription([mega, uno])
