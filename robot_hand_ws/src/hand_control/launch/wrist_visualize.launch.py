"""One dynamic wrist TF publisher; the URDF publishes finger curl and spread TF."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = LaunchConfiguration('model')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(
            get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8766'),
        DeclareLaunchArgument('alpha', default_value='0.55'),
        DeclareLaunchArgument('motion_speed_scale', default_value='2.5'),
        DeclareLaunchArgument('wrist_tau', default_value='0.06'),
        DeclareLaunchArgument('wrist_max_speed', default_value='6.0'),
        DeclareLaunchArgument('wrist_timeout', default_value='0.5'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command([FindExecutable(name='xacro'), ' ', model]), value_type=str)}],
             output='screen'),
        Node(package='hand_control', executable='wrist_receiver', output='screen',
             parameters=[{'model': model, 'host': LaunchConfiguration('host'),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                             for name in ('alpha', 'motion_speed_scale', 'wrist_tau', 'wrist_max_speed', 'wrist_timeout')}}]),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(get_package_share_path('hand_control') / 'rviz/wrist.rviz')],
             output='screen'),
    ])
