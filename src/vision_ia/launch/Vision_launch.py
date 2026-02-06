from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1️⃣ Nœud caméra ---
    camera_node = Node(
        package='vision_ia',
        executable='camera',
        name='camera_node',
        output='screen',
        parameters=[
            # --- Configuration Caméra 1 ---
            {'camera_index_1': 0},
            {'topic_name_1': '/video_cam'},
            
            # --- Configuration Caméra 2 ---
            {'camera_index_2': 2},        # L'index validé avec ffplay
            {'topic_name_2': '/video_cam_2'},
        ]
    )

    # --- 2️⃣ Nœud YOLO / suivi de personne ---
    vision_ia_node = Node(
        package='vision_ia',
        executable='vision_IA',
        name='vision_IA_node',
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        vision_ia_node,
    ])
