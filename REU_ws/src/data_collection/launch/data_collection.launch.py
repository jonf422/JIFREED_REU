from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='data_collection',
            executable='realsense_node',
            name='realsense_node',
            output='screen',
        ),

        Node(
            package='data_collection',
            executable='arducam_node',
            name='arducam_node',
            output='screen',
        ),

        Node(
            package='data_collection',
            executable='data_save',
            name='data_collection_node',
            output='screen',
            prefix='xterm -e',
        ),

    ])