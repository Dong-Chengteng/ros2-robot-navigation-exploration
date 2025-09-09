from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    world_path = '/home/dong/ros2_ws/src/first_pkg/map/world_5_19.world'
    model_path = '/home/dong/ros2_ws/src/first_pkg/models/aerial_lidar/model.sdf'

    return LaunchDescription([
        # 启动 Gazebo 并加载你的 world
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # 向 world 中加载雷达模型
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'aerial_lidar',
                '-file', model_path,
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )
    ])