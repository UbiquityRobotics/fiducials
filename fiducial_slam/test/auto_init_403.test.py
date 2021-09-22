import unittest

import launch
import launch_ros
import launch_testing
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import pytest

@pytest.mark.rostest
def generate_test_description():
    test_node = launch_ros.actions.Node(
        executable=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), 'auto_init_403_test']),
        parameters=[
            {"image_directory":get_package_share_directory('fiducial_slam') + '/test/test_images/'}
        ],
        output='screen',
    )

    static_tranform_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base',
        arguments="0.035 0.145 0.14 -1.479119 -0.041544 -1.204205 base_link camera".split()
    )

    aruco_detect = launch_ros.actions.Node(
        package='aruco_detect',
        namespace='aruco_detect',
        executable='aruco_detect',
        name='aruco_detect',
        parameters=[
            {"image_transport":'raw'},
            {"publish_images":True},
            {"fiducial_len":0.145},
        ],
        #Not sure if the remapping is working properly TODO
        remappings=[
            ('camera/compressed', 'camera/image/compressed'),
            ('fiducial_vertices', '/fiducial_vertices'),
            ('fiducial_transforms', '/fiducial_transforms')
        ]
    )

    fiducial_slam = launch_ros.actions.Node(
        package='fiducial_slam',
        executable='fiducial_slam_node',
        name='fiducial_slam_node',
        parameters=[
            {"map_frame":'map'},
            {"odom_frame":""},
            {"base_frame":"base_link"},
            {"future_date_transforms", "0.0"},
            {"publish_6dof_pose","true"},
            {"map_file":"/dev/null"}
        ],
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir', # From https://github.com/ros-planning/moveit2/blob/1e1337b46daed0aaa1252b87367c1824f46c9b1a/moveit_ros/moveit_servo/test/launch/servo_launch_test_common.py#L92
                                             description='Binary directory of package containing test executables'),
        # other fixture actions
        static_tranform_publisher,
        aruco_detect,
        fiducial_slam,
        test_node,
        #launch_testing.util.KeepAliveProc(), # From https://github.com/ros2/geometry2/blob/a1be44fdbd8c032351ffd3ddff8467d000f4c6d7/test_tf2/test/buffer_client_tester.launch.py#L39
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class TestTestTermination(unittest.TestCase):
    def test_termination(self, test_node, proc_output, proc_info):
        proc_info.assertWaitForShutdown(process=test_node, timeout=(30))

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_no_failed_tests(self, proc_output, proc_info, test_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        number_of_failed_tests = 0
        launch_testing.asserts.assertExitCodes(proc_info, [number_of_failed_tests], process=test_node)