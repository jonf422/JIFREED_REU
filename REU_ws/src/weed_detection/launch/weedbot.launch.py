'''
Jonathan Freed

weedbot_launch.py

Robot Launch File

Parameters:
        headless_launch     Bool - Launch without display (Default: False)
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    headless_launch = LaunchConfiguration('headless_launch')
    gps_off = LaunchConfiguration('gps_off')
    without_mission = LaunchConfiguration('without_mission')

    coords = LaunchConfiguration('coords')
    frame = LaunchConfiguration('frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless_launch',
            default_value='false',
            description='If true, skip display node'
        ),
        DeclareLaunchArgument(
            'gps_off',
            default_value='false',
            description='If true, skip nmea_serial_driver'
        ),
        DeclareLaunchArgument(
            'without_mission',
            default_value='false',
            description='If true, skip mission_manager_node'
        ),

        DeclareLaunchArgument(
            'coords',
            default_value='[0.0,0.0]',
            description='Target Waypoint [x,y] or [lat,lon] (MAX RANGE default = 20 m). Default waypoint [0.0,0.0] will skip waypoint navigation)'
        ),
        DeclareLaunchArgument(
            'frame',
            default_value='map',
            description='Default frame for coordinates (map or odom)'
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
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            ),
        Node(
            package='weed_detection',
            executable='wheel_odom_node',
            name='wheel_odom_node',
        ),
        Node( #Only if outside -> if inside set gps_off arg to true
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{
                'port': '/dev/rtk_gps',
                'baud': 115200,
            }],
            condition=UnlessCondition(gps_off),
        ),
        # --- Control -------------------------------
        Node(
            package='weed_detection',
            executable='drive_controller_node',
            name='drive_controller_node',
        ),
        Node(
            package='weed_detection',
            executable='mission_manager_node',
            name='mission_manager_node',
            parameters=[{'coords': ParameterValue(coords, value_type=list[float]),
                         'frame': ParameterValue(frame, value_type=str)}],
            condition=UnlessCondition(without_mission),
        ),

    ])