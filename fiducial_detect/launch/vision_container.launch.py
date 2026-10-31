import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    detector_type_arg = DeclareLaunchArgument(
        'detector_type', default_value='aruco',
        description='Detector plugin type: "aruco" or "stag"'
    )
    fiducial_len_arg = DeclareLaunchArgument(
        'fiducial_len', default_value='0.14',
        description='Default physical marker side length in meters'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Broadcast TF transforms for detected markers'
    )

    container = ComposableNodeContainer(
        name='fiducial_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='fiducial_detect',
                plugin='fiducial_detect::ArucoDetectComponent',
                name='aruco_detector',
                parameters=[{
                    'fiducial_len': LaunchConfiguration('fiducial_len'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'use_intra_process_comms': True
                }],
                remappings=[
                    ('image_raw', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        detector_type_arg,
        fiducial_len_arg,
        publish_tf_arg,
        container
    ])
