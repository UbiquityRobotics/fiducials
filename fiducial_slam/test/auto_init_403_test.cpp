/*
Test to verify that when the map is initialized from
a single fiducial that the robot's pose is identity
and the map entry for the fiducial is reasonable
*/

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "fiducial_msgs/msg/fiducial_map_entry.hpp"
#include "fiducial_msgs/msg/fiducial_map_entry_array.hpp"


class AutoInitTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    nh = std::make_shared<rclcpp::Node>("auto_init_403_test");

    // Declare the parameters
    image_directory = nh->declare_parameter("image_directory", "");

    it = std::make_shared<image_transport::ImageTransport>(nh);
    image_pub = it->advertise("camera/image", 1);

    camera_info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 5);

    c_info.header.frame_id = "camera";
    c_info.height = 960;
    c_info.width = 1280;
    c_info.distortion_model = "plumb_bob";

    double data_D[] = {0.1349735087283542, -0.2335869827451621,
                       0.0006697030315075139, 0.004846737465872353, 0.0};

    c_info.d = std::vector<double> (data_D,
                                    data_D + sizeof(data_D)/ sizeof(double));

    c_info.k = {1006.126285753055, 0.0,               655.8639244150409, 
                0.0,               1004.015433012594, 490.6140221242933,
                0.0,               0.0,               1.0};

    c_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    c_info.p = {1021.54345703125, 0.0,            661.9091982335958, 0.0,
                0.0,              1025.251953125, 490.6380671707448, 0.0,
                0.0,              0.0,            1.0,               0.0};

    map_sub = nh->create_subscription<fiducial_msgs::msg::FiducialMapEntryArray>
        ("/fiducial_map", 1, std::bind(&AutoInitTest::map_callback, this, std::placeholders::_1));


    pose_sub = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
        ("/fiducial_pose", 1, std::bind(&AutoInitTest::pose_callback, this, std::placeholders::_1));
  }

  void publish_image(std::string file)
  {
    cv::Mat image = cv::imread(image_directory+file, cv::IMREAD_COLOR);

    if(image.data == NULL)
    {
      FAIL() << "Cannot open " << image_directory+file;
    }
    //cv::waitKey(30); // Should not be needed
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    c_info.header.stamp = nh->get_clock()->now();
    camera_info_pub->publish(c_info);
  }

  void map_callback(const fiducial_msgs::msg::FiducialMapEntryArray::SharedPtr msg)
  {
    got_map = true;
    map = msg;
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    got_pose = true;
    pose = msg;
  }

  rclcpp::Node::SharedPtr nh;

  // Set up Publishing of static images
  std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher image_pub;

  sensor_msgs::msg::CameraInfo c_info;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

  std::string image_directory;

  rclcpp::Subscription<fiducial_msgs::msg::FiducialMapEntryArray>::SharedPtr map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;

  bool got_map{false};
  bool got_pose{false};

  fiducial_msgs::msg::FiducialMapEntryArray::SharedPtr map;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose;

};

TEST_F(AutoInitTest, tag_403_d7_14cm) {
  int loop_count = 0;
  rclcpp::Rate loop_rate(5);
  while (rclcpp::ok() && (!got_pose || !got_map)) {
    publish_image("403.jpg");
    rclcpp::spin_some(nh);
    loop_rate.sleep();
    loop_count++;
    if (loop_count > 100) {
      FAIL() << "Did not receive estimate within 10 frames";
    }
  }

  ASSERT_NEAR(0, pose->pose.pose.position.x, 0.001);
  ASSERT_NEAR(0, pose->pose.pose.position.y, 0.001);
  ASSERT_NEAR(0, pose->pose.pose.position.z, 0.001);

  ASSERT_NEAR(1, pose->pose.pose.orientation.w, 0.001);
  ASSERT_NEAR(0, pose->pose.pose.orientation.x, 0.001);
  ASSERT_NEAR(0, pose->pose.pose.orientation.y, 0.001);
  ASSERT_NEAR(0, pose->pose.pose.orientation.z, 0.001);

  ASSERT_LE(1, map->fiducials.size());

  const fiducial_msgs::msg::FiducialMapEntry &fid = map->fiducials[0];
  ASSERT_EQ(403, fid.fiducial_id);
  ASSERT_NEAR(0.7611, fid.x, 0.001);
  ASSERT_NEAR(0.2505, fid.y, 0.001);
  ASSERT_NEAR(0.4028, fid.z, 0.001);
  ASSERT_NEAR(1.5751, fid.rx, 0.001);
  ASSERT_NEAR(-0.014, fid.ry, 0.001);
  ASSERT_NEAR(-1.546, fid.rz, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
