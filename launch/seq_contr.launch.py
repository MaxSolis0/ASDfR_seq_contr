from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Get current working directory (should be /ros2_ws)
    ws_root = os.getcwd()
    ws_src = os.path.join(ws_root, 'src')

    return LaunchDescription([

        # =============================
        # 1) cam2image node
        # =============================
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            output='screen',
            parameters=[
                os.path.join(
                    ws_src,
                    'cam2image_vm2ros',
                    'config',
                    'cam2image.yaml'
                )
            ]
        ),

        # =============================
        # 2) color_tracker node
        # =============================
        Node(
            package='img_proc',
            executable='color_tracker',
            name='color_tracker',
            output='screen',
            parameters=[
                os.path.join(
                    ws_src,
                    'image_proc',
                    'config',
                    'color_tracker.yaml'
                )
            ],
            remappings=[
                ('image', 'output/moving_camera')
            ]
        ),

        # =============================
        # 3) RELbot simulator
        # =============================
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            output='screen'
        ),

        # =============================
        # 4) Setpoint sequence node
        # =============================
        Node(
            package='seq_contr',
            executable='setpoint_sequence_node',
            name='setpoint_sequence_node',
            output='screen',
            parameters=[
                os.path.join(
                    ws_src,
                    'seq_contr',
                    'config',
                    'setpoints.yaml'
                )
            ]
        ),

        # =============================
        # 5) 2D plotter
        # =============================
        Node(
            package='seq_contr',
            executable='plotter2D.py',
            name='plotter2D',
            output='screen'
        ),

    ])
