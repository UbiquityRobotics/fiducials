from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('stag_detect')
    single_yaml_path = PathJoinSubstitution([package_share, 'cfg', 'single.yaml'])

    # Declare the launch argument
    fiducial_transform_topic_arg = DeclareLaunchArgument(
        'fiducial_transform_topic',
        default_value='/fiducial_transforms',
        description='Fiducial transform topic override'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.18',
        description='Size of the fiducial marker in meters'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Optional relative path inside the stag_detect share directory to a YAML file with per-marker sizes'
    )

    def create_stag_node(context):
        config_file = LaunchConfiguration('config_file').perform(context)
        if config_file and not os.path.isabs(config_file):
            resolved_config_file = os.path.join(package_share, config_file)
        else:
            resolved_config_file = config_file

        stag_detect_node = Node(
            package='stag_detect',
            executable='stag_detect',
            name='stag_detect',
            output='screen',
            parameters=[
                {'marker_size': LaunchConfiguration('marker_size')},
                {'config_file': resolved_config_file},
                {'stag_library': 11},
                {'image_topic': "/camera/image_raw/compressed"},
                {'is_compressed': True},
                {'camera_info_topic': "/camera/camera_info"},
                single_yaml_path  # Load parameters from YAML file
            ],
            remappings=[
                ('stag_ros/markers_array', LaunchConfiguration('fiducial_transform_topic'))
            ],
        )
        return [stag_detect_node]

    return LaunchDescription([
        fiducial_transform_topic_arg,
        marker_size_arg,
        config_file_arg,
        OpaqueFunction(function=create_stag_node),
    ])
