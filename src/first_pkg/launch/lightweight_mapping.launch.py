from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # 启动轻量级SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='lightweight_mapper',
            parameters=[{
                'use_sim_time': use_sim_time,
                
                # 基本参数 - 最小配置
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'use_map_saver': True,
                'mode': 'mapping',
                
                # 传感器范围设置
                'min_laser_range': 0.12,
                'max_laser_range': 8.0,
                
                # 高性能设置 - 解决消息队列溢出问题
                'map_publish_period': 0.1,   # 降低发布频率到100毫秒
                'map_update_interval': 0.1,  # 降低更新频率到100毫秒
                'debug_logging': False,      # 关闭调试日志
                'throttle_scans': 2,         # 跳过一些扫描以减轻负载
                'transform_publish_period': 0.1,  # 降低变换发布频率
                'resolution': 0.05,          # 地图分辨率保持0.05米/像素
                'minimum_time_interval': 0.1,    # 100毫秒最小间隔
                'minimum_travel_distance': 0.15, # 增加到150毫米才更新
                'minimum_travel_heading': 0.15,  # 增加到15度才更新
                'transform_timeout': 0.5,    # 增加超时时间
                'tf_buffer_duration': 3.0,   # 减少缓冲区大小
                'stack_size_to_use': 10000000, # 减少栈大小
                'enable_interactive_mode': False,
                
                # 消息队列优化 - 大幅增加缓冲区
                'max_queue_size': 200,       # 大幅增加队列大小
                'scan_buffer_size': 100,     # 增加扫描缓冲区
                'scan_buffer_maximum_scan_distance': 6.0,
                
                # Ceres求解器优化
                'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_preconditioner': 'SCHUR_JACOBI',
                'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                'ceres_loss_function': 'None',
                'ceres_num_threads': 16,     # 降低线程数以避免超出系统限制
                
                # 算法参数优化
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'do_loop_closing': False,    # 临时关闭环闭检测以减轻计算负载
                'loop_match_maximum_variance_coarse': 0.4,
                'loop_match_minimum_response_coarse': 0.3,
                'loop_match_minimum_response_fine': 0.4,
                
                # 额外的性能优化参数
                'laser_sample_size': 70,     # 减少激光采样点数
                'correlation_search_space_dimension': 0.3,
                'correlation_search_space_resolution': 0.01,
                'correlation_search_space_smear_deviation': 0.03,
                'fine_search_angle_offset': 0.00349,
                'coarse_search_angle_offset': 0.349,
                'coarse_angle_resolution': 0.0349,
                'minimum_angle_penalty': 0.9,
                'minimum_distance_penalty': 0.5,
            }],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ],
            output='screen',
        ),
    ])
