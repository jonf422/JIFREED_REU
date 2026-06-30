'''
Jonathan Freed

tf_tree_launch.py

TF Tree Launch file

'''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

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
            arguments = ['0', '0', '.42',
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
    ])