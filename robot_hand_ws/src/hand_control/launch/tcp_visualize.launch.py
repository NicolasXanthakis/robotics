"""Visualize TCP-controlled hand motion without starting the demo publisher."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Reuse the existing visualization with its demonstration motion disabled."""
    share = get_package_share_path('hand_control')
    model = LaunchConfiguration('model')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(
            get_package_share_path('robot_hand_description') / 'urdf/hand.urdf.xacro')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8766'),
        DeclareLaunchArgument('alpha', default_value='0.2'),
        DeclareLaunchArgument('stale_timeout', default_value='1.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/visualize.launch.py')),
            launch_arguments={'model': model, 'rviz': LaunchConfiguration('rviz'),
                              'test_motion': 'false'}.items()),
        Node(package='hand_control', executable='webcam_receiver', output='screen',
             parameters=[{'model': model, 'host': LaunchConfiguration('host'),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          'alpha': ParameterValue(LaunchConfiguration('alpha'), value_type=float),
                          'stale_timeout': ParameterValue(
                              LaunchConfiguration('stale_timeout'), value_type=float)}]),
    ])
