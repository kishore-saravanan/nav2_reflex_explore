from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_reflex_explore')

    params_file = LaunchConfiguration('params_file')
    map_save_path = LaunchConfiguration('map_save_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'params', 'explorer_params.yaml']),
            description='Path to explorer params YAML.'
        ),
        DeclareLaunchArgument(
            'map_save_path',
            default_value='',
            description='Absolute base path (no extension) for map saving.'
        ),

        Node(
            package='nav2_reflex_explore',
            executable='reflex_explorer',
            name='reflex_explorer',
            output='screen',
            parameters=[
                ParameterFile(params_file, allow_substs=True),
                {'map_save_path': map_save_path},
            ],
        ),
    ])
