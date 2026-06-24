'''
Jonathan Freed

weedbot.launch.py

Robot Launch File

Parameters:
        headless_launch     Bool - Launch without display (Default: False)
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    headless_launch = LaunchConfiguration('headless_launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless_launch',
            default_value='false',
            description='If true, skip display node'
        ),

        # --- Display - only when NOT headless--------
        Node(
            package='weed_detection',
            executable='combined_display_node',
            name='combined_display_node',
            condition=UnlessCondition(headless_launch),
        ),
        # --- Vision --------------------------------
        Node(
            package='weed_detection',
            executable='realsense_multistream_node',
            name='realsense_multistream_node',
        ),
        Node(
            package='weed_detection',
            executable='arducam_node',
            name='arducam_node',
        ),
        # --- Navigation -----------------------------
        Node(
            package='weed_detection',
            executable='wheel_odom_node',
            name='wheel_odom_node',
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
            }],
        ),
        # --- Control -------------------------------
        Node(
            package='weed_detection',
            executable='drive_controller_node',
            name='drive_controller_node',
        ),

    ])