from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_share = FindPackageShare('stag_detect')
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

    # Node definition
    stag_detect_node = Node(
        package='stag_detect',
        executable='stag_detect',
        name='stag_detect',
        output='screen',
        parameters=[
            {'marker_size': LaunchConfiguration('marker_size')},
            {'config_file': LaunchConfiguration('config_file')},
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

    return LaunchDescription([
        fiducial_transform_topic_arg,
        marker_size_arg,
        config_file_arg,
        stag_detect_node,
    ])
