#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR publisher node
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',  # 已经是新版API
        name='MS200',              # 已经是新版API
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            # {'frame_id': 'laser_frame'},
            # {'scan_topic': 'MS200/scan'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': 'scan'},
            {'port_name': '/dev/ttyACM0'},
            {'baudrate': 230400},
            {'angle_min': 90.0},
            {'angle_max': 270.0},
            {'range_min': 0.20},
            {'range_max': 20.0},
            {'clockwise': True},
            {'motor_speed': 10}
        ]
    )

    # base_link to laser_frame tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',  # 修改：node_executable -> executable
        name='base_link_to_base_laser',          # 修改：node_name -> name
        arguments=['-0.0019517', '0.00040736', '0.12059', '-3.1416', '0', '0', 'base_link', 'lidar_link']
    )

    # Define LaunchDescription variable
    ord = LaunchDescription()

    ord.add_action(ordlidar_node)
    # ord.add_action(base_link_to_laser_tf_node)  # 保持注释状态

    return ord