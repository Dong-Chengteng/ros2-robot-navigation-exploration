import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions


def generate_launch_description():
    #获取默认urdf路径
    urdf_package_path = get_package_share_directory('first_pkg')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot','dong_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')

    #生成urdf目录参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model', default_value=str(default_urdf_path), description='加载的模型文件路径'
    )

    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)

    #启动节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':robot_description_value}]
    )
    action_joint_state_publisher = launch_ros.actions.Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
    )
    action_rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d , default_rviz_config_path']
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])
