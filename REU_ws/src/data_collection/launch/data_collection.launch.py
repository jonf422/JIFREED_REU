from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RealSense camera — color + aligned depth publisher
        Node(
            package='data_collection',
            executable='realsense_node',
            name='realsense_node',
            output='screen',
        ),

        # Arducam — RGB publisher
        Node(
            package='data_collection',
            executable='arducam_node',
            name='arducam_node',
            output='screen',
        ),

        # Teleop — own terminal for isolated keyboard focus
        Node(
            package='data_collection',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            prefix='xterm -e',
        ),

        # Data collection — own terminal for keyboard input + OpenCV windows
        Node(
            package='data_collection',
            executable='data_save',
            name='data_collection_node',
            output='screen',
            prefix='xterm -e',
        ),

    ])