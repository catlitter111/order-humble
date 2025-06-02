#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取slam_toolbox的配置文件路径
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # MS200激光雷达节点
    ms200_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'laser'},
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

    # TF变换：base_link到laser
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
    )

    # SLAM Toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'solver_plugin': 'solver_plugins::CeresSolver'},
            {'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY'},
            {'ceres_preconditioner': 'SCHUR_JACOBI'},
            {'ceres_trust_strategy': 'LEVENBERG_MARQUARDT'},
            {'ceres_dogleg_type': 'TRADITIONAL_DOGLEG'},
            {'ceres_loss_function': 'None'},
            {'odom_frame': 'odom'},
            {'map_frame': 'map'},
            {'base_frame': 'base_link'},
            {'scan_topic': '/scan'},
            {'mode': 'mapping'},
            {'debug_logging': False},
            {'throttle_scans': 1},
            {'transform_publish_period': 0.02},
            {'map_update_interval': 5.0},
            {'resolution': 0.05},
            {'max_laser_range': 20.0},
            {'minimum_time_interval': 0.5},
            {'transform_timeout': 0.2},
            {'tf_buffer_duration': 30.0},
            {'stack_size_to_use': 40000000},
            {'enable_interactive_mode': True}
        ]
    )

    # RViz2可视化
    rviz_config_file = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'rviz2',
        'slam.rviz'
    )
    
    # 如果没有专用配置文件，使用默认参数
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    return LaunchDescription([
        use_sim_time_arg,
        ms200_node,
        base_to_laser_tf,
        slam_toolbox_node,
        rviz_node
    ])