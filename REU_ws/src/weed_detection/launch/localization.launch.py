'''
Jonathan Freed

localization.launch.py

TF Tree + Robot Localization Launch file

'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gps_off = LaunchConfiguration('gps_off')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gps_off',
            default_value='false',
            description='If true, skip ekf_filter_node_map, navsat_transform'
        ),
    #---------------------------------------------------
    #Transform Tree
    #---------------------------------------------------
        # --- Base Link -> Sensors (Static) ------------ 
        
        #argument format: 
        #        (x,y,z),
        #        (yaw,pitch,roll),
        #        parent_frame, child_frame     
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_realsense',
            arguments = ['.215', '0', '.56',
                         '0', '.611', '0',
                         'base_link', 'realsense_link'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='realsense_to_optical',
            arguments=['0','0','0', '-1.5708','0','-1.5708',
                    'realsense_link', 'realsense_optical'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_arducam',
            arguments = ['.1475', '0', '.07',
                         '0', '0', '0',
                         'base_link', 'arducam_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments = ['.1475', '0', '.56',
                         '0', '0', '0',
                         'base_link', 'gps_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_drill',
            arguments = ['.2425', '0', '0',
                         '0', '0', '0',
                         'base_link', 'drill_link'],
        ),

    #---------------------------------------------------
    #Robot Localization
    #---------------------------------------------------
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[os.path.join(get_package_share_directory('weed_detection'), 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', 'odometry/local')],
    ),
    
    #Skip if gps_off == true
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[os.path.join(get_package_share_directory('weed_detection'), 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', 'odometry/global')],
        condition=UnlessCondition(gps_off),
    ),
    
    Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[os.path.join(get_package_share_directory('weed_detection'), 'config', 'ekf.yaml')],
        remappings=[('gps/fix', '/fix'),
                    ('imu', '/gps/heading'),
                    ('odometry/filtered', 'odometry/global'),
                    ],
        condition=UnlessCondition(gps_off),
        ),
        
    ])