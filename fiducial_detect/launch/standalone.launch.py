import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fiducial_len_arg = DeclareLaunchArgument(
        'fiducial_len', default_value='0.14',
        description='Default physical marker side length in meters'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Broadcast TF transforms for detected markers'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/color/image_raw',
        description='Input camera image topic'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info',
        description='Input camera info topic'
    )

    aruco_node = Node(
        package='fiducial_detect',
        executable='aruco_detector_node',
        name='aruco_detector',
        parameters=[{
            'fiducial_len': LaunchConfiguration('fiducial_len'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
        remappings=[
            ('image_raw', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic'))
        ],
        output='screen'
    )

    return LaunchDescription([
        fiducial_len_arg,
        publish_tf_arg,
        image_topic_arg,
        camera_info_topic_arg,
        aruco_node
    ])
