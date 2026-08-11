'''
Jonathan Freed

weedbot_launch.py

Robot Launch File

Parameters:
        headless_launch     Bool - Launch without display (Default: False)
        coords              Float list - Ordered waypoints, flat pairs: [x1,y1,x2,y2,...]
                            (Default: [0.0,0.0] = skip navigation)
        frame               Str - Frame for coords, 'odom' or 'map' (Default: odom)

Example:
        ros2 launch weed_detection weedbot.launch.py coords:="[3.0,0.0, 3.0,2.0, 0.0,2.0]"
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import List

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
            description='Ordered target waypoints as a flat list of pairs: [x1,y1,x2,y2,...] or '
                        '[lat1,lon1,lat2,lon2,...]. Each leg is sent as its own goal and must be '
                        'within MAX RANGE (default 20 m) of the robot when that leg starts. '
                        'The default [0.0,0.0] skips waypoint navigation.'
        ),
        DeclareLaunchArgument(
            'frame',
            default_value='odom',
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
        Node( #Converts nmea_navsat_driver's /heading (dual-antenna RTK) to a usable absolute yaw
            package='weed_detection',
            executable='gps_heading_node',
            name='gps_heading_node',
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
            parameters=[{'coords': ParameterValue(coords, value_type=List[float]),
                         'frame': ParameterValue(frame, value_type=str)}],
            condition=UnlessCondition(without_mission),
        ),

    ])
