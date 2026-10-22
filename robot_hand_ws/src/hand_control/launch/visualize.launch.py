"""Launch the hand model, demonstration motion, and RViz."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Use the same installed Xacro for visualization and joint limits."""
    model = LaunchConfiguration('model')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=str(
                get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro'
            )),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('test_motion', default_value='true'),
        DeclareLaunchArgument('period', default_value='8.0'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', model]), value_type=str)}],
            output='screen'),
        Node(
            package='hand_control', executable='test_joint_publisher',
            condition=IfCondition(LaunchConfiguration('test_motion')),
            parameters=[{'model': model, 'period': ParameterValue(
                LaunchConfiguration('period'), value_type=float)}],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(
                get_package_share_path('hand_control') / 'rviz/hand.rviz'
            )], output='screen'),
    ])
