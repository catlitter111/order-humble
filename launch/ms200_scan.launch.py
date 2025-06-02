#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR publisher node
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',  # 修改：node_executable -> executable
        name='MS200',              # 修改：node_name -> name
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/oradar'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ]
    )

    # base_link to laser_frame tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',  # 修改：node_executable -> executable
        name='base_link_to_base_laser',          # 修改：node_name -> name
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'lidar']
    )

    # Define LaunchDescription variable
    ord = LaunchDescription()

    ord.add_action(ordlidar_node)
    ord.add_action(base_link_to_laser_tf_node)

    return ord