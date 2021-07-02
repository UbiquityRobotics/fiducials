from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/camera'),
        DeclareLaunchArgument('use_fiducial_area_as_weight', default_value='false'),
        DeclareLaunchArgument('weighting_scale', default_value='1e9'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_interval', default_value='0.2'),
        DeclareLaunchArgument('future_date_transforms', default_value='0.0'),
        DeclareLaunchArgument('publish_6dof_pose', default_value='false'),
        DeclareLaunchArgument('systematic_error', default_value='0.01'),
        DeclareLaunchArgument('covariance_diagonal', default_value=''),
        DeclareLaunchArgument('read_only_map', default_value='False'),
        Node(
            package='fiducial_slam',
            namespace='fiducial_slam',
            executable='fiducial_slam_node',
            name='fiducial_slam_node',
            parameters=[
                {"multi_error_theshold": -1.0},
                {"map_file":[EnvironmentVariable('HOME'), "/.ros/slam/map.txt"]},
                {"initial_map_file":""},
                {"use_fiducial_area_as_weight":LaunchConfiguration('use_fiducial_area_as_weight')},
                {"weighting_scale":LaunchConfiguration('weighting_scale')},
                {"map_frame":LaunchConfiguration('map_frame')},
                {"odom_frame":LaunchConfiguration('odom_frame')},
                {"base_frame":LaunchConfiguration('base_frame')},
                {"publish_tf":LaunchConfiguration('publish_tf')},
                {"tf_publish_interval":LaunchConfiguration('tf_publish_interval')},
                {"future_date_transforms":LaunchConfiguration('future_date_transforms')},
                {"publish_6dof_pose":LaunchConfiguration('publish_6dof_pose')},
                {"read_only_map":LaunchConfiguration('read_only_map')},
                {"covariance_diagonal":LaunchConfiguration('covariance_diagonal')}
            ],
            remappings=[
                ('/fiducial_transforms', '/aruco_detect/fiducial_transforms'),
            ]
        )
    ])
