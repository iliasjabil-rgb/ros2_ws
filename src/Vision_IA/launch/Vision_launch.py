from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1️⃣ Nœud caméra ---
    camera_node = Node(
        package='Vision_IA',
        executable='camera',
        name='camera_node',
        output='screen'
    )

    # --- 2️⃣ Nœud YOLO / suivi de personne ---
    vision_ia_node = Node(
        package='Vision_IA',
        executable='vision_IA',
        name='vision_IA_node',
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        vision_ia_node,
    ])
