from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('insta360_x5_ros2')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'insta360_x5.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('namespace', default_value='insta360_x5'),
        Node(
            package='insta360_x5_ros2',
            executable='insta360_x5_node',
            name='insta360_x5_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
    ])
