#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <iostream>
#include <chrono>
#include <thread>

#include <image_transport/image_transport.hpp>

// #include <image_transport/image_transport.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"

class ArucoImagesTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    nh = std::make_shared<rclcpp::Node>("aruco_images_test");

    // Declare the parameters
    image_directory = nh->declare_parameter("image_directory", "");

    it = std::make_shared<image_transport::ImageTransport>(nh);
    image_pub = it->advertise("camera/image", 1);

    camera_info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 5);

    c_info.height = 960;
    c_info.width = 1280;
    c_info.distortion_model = "plumb_bob";

    double data_D[] = {0.1349735087283542, -0.2335869827451621, 0.0006697030315075139, 0.004846737465872353, 0.0};
    c_info.d = std::vector<double> (data_D, data_D + sizeof(data_D)/ sizeof(double));

    c_info.k = {1006.126285753055, 0.0, 655.8639244150409, 0.0, 1004.015433012594, 490.6140221242933, 0.0, 0.0, 1.0};
    c_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    c_info.p = {1021.54345703125, 0.0, 661.9091982335958, 0.0, 0.0, 1025.251953125, 490.6380671707448, 0.0, 0.0, 0.0, 1.0, 0.0};

    vertices_sub = nh->create_subscription<fiducial_msgs::msg::FiducialArray>
        ("/fiducial_vertices", 1, std::bind(&ArucoImagesTest::vertices_callback, this, std::placeholders::_1));

    transforms_sub = nh->create_subscription<fiducial_msgs::msg::FiducialTransformArray>
        ("/fiducial_transforms", 1, std::bind(&ArucoImagesTest::transforms_callback, this, std::placeholders::_1));
  }

  void publish_image(std::string file) {
    cv::Mat image = cv::imread(image_directory+file, cv::IMREAD_COLOR);

    if(image.data == NULL)
    {
      FAIL() << "Cannot open " << image_directory+file;
    }
    //cv::waitKey(30); // Should not be needed
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    RCLCPP_ERROR(nh->get_logger(), "Publish image!");
    c_info.header.stamp = nh->get_clock()->now();
    camera_info_pub->publish(c_info);
  }

  void vertices_callback(const fiducial_msgs::msg::FiducialArray::SharedPtr f) {
    got_vertices = true;
    fiducials = f;
  }

  void transforms_callback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr f) {
    got_transforms = true;
    fiducial_tfs = f;
  }

  rclcpp::Node::SharedPtr nh;
  rclcpp::Node::SharedPtr pnh;

  // // Set up Publishing of static images
  std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher image_pub;

  sensor_msgs::msg::CameraInfo c_info;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

  std::string image_directory;

  // // Set up subscribing
  bool got_vertices {false};
  fiducial_msgs::msg::FiducialArray::SharedPtr fiducials;
  rclcpp::Subscription<fiducial_msgs::msg::FiducialArray>::SharedPtr vertices_sub;

  bool got_transforms {false};
  fiducial_msgs::msg::FiducialTransformArray::SharedPtr fiducial_tfs;
  rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr transforms_sub;
};

TEST_F(ArucoImagesTest, tag_01_d7_14cm) {
  int loop_count = 0;
  rclcpp::Rate loop_rate(5);
  while (rclcpp::ok() && (!got_vertices || !got_transforms)) {
    publish_image("tag_01_d7_14cm.png");
    rclcpp::spin_some(nh);
    loop_rate.sleep();
    loop_count++;
    if (loop_count > 10) {
      FAIL() << "Did not receive estimate within 10 frames";
    }
  }

  ASSERT_EQ(1, fiducials->fiducials.size());
  ASSERT_EQ(1, fiducial_tfs->transforms.size());

  const fiducial_msgs::msg::Fiducial vertices = fiducials->fiducials[0];
  ASSERT_EQ(1, vertices.fiducial_id);

  ASSERT_FLOAT_EQ(569.89917, vertices.x0);
  ASSERT_FLOAT_EQ(201.55890, vertices.y0);
  ASSERT_FLOAT_EQ(777.42560, vertices.x1);
  ASSERT_FLOAT_EQ(206.85025, vertices.y1);
  ASSERT_FLOAT_EQ(767.95856, vertices.x2);
  ASSERT_FLOAT_EQ(415.37830, vertices.y2);
  ASSERT_FLOAT_EQ(565.75311, vertices.x3);
  ASSERT_FLOAT_EQ(409.24496, vertices.y3);
}

TEST_F(ArucoImagesTest, tag_245_246_d7_14cm) {
  int loop_count = 0;
  rclcpp::Rate loop_rate(5);
  while (rclcpp::ok() && (!got_vertices || !got_transforms)) {
    publish_image("tag_245-246_d7_14cm.png");
    rclcpp::spin_some(nh);
    loop_rate.sleep();
    loop_count++;
    if (loop_count > 10) {
      FAIL() << "Did not receive estimate within 10 frames";
    }
  }

  ASSERT_EQ(2, fiducials->fiducials.size());
  ASSERT_EQ(2, fiducial_tfs->transforms.size());

  for (auto& vertices: fiducials->fiducials) {
    if (vertices.fiducial_id == 245) {
      ASSERT_FLOAT_EQ(307.68246, vertices.x0);
      ASSERT_FLOAT_EQ(157.38346, vertices.y0);
      ASSERT_FLOAT_EQ(545.10131, vertices.x1);
      ASSERT_FLOAT_EQ(167.04420, vertices.y1);
      ASSERT_FLOAT_EQ(540.11614, vertices.x2);
      ASSERT_FLOAT_EQ(403.27578, vertices.y2);
      ASSERT_FLOAT_EQ(305.64746, vertices.x3);
      ASSERT_FLOAT_EQ(395.01422, vertices.y3);
    }
    else  if (vertices.fiducial_id == 246) {
      ASSERT_FLOAT_EQ(671.51892, vertices.x0);
      ASSERT_FLOAT_EQ(173.46070, vertices.y0);
      ASSERT_FLOAT_EQ(900.29650, vertices.x1);
      ASSERT_FLOAT_EQ(178.44973, vertices.y1);
      ASSERT_FLOAT_EQ(895.06933, vertices.x2);
      ASSERT_FLOAT_EQ(407.39855, vertices.y2);
      ASSERT_FLOAT_EQ(666.39910, vertices.x3);
      ASSERT_FLOAT_EQ(403.12911, vertices.y3);
    }
    else {
      FAIL();
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
