#include <gtest/gtest.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>

class ArucoImagesTest : public ::testing::Test {
protected:
  virtual void SetUp() { 
    it = new image_transport::ImageTransport(nh);
    image_pub = it->advertise("camera/image", 1);

    CameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 5);

    c_info.height = 960;
    c_info.width = 1280;
    c_info.distortion_model = "plumb_bob";

    double data_D[] = {0.1349735087283542, -0.2335869827451621, 0.0006697030315075139, 0.004846737465872353, 0.0};
    c_info.D = std::vector<double> (data_D, data_D + sizeof(data_D)/ sizeof(double));
   
    c_info.K = {1006.126285753055, 0.0, 655.8639244150409, 0.0, 1004.015433012594, 490.6140221242933, 0.0, 0.0, 1.0};
    c_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    c_info.P = {1021.54345703125, 0.0, 661.9091982335958, 0.0, 0.0, 1025.251953125, 490.6380671707448, 0.0, 0.0, 0.0, 1.0, 0.0};

    ros::NodeHandle nh_priv("~");
    nh_priv.getParam("image_directory", image_directory);

    vertices_sub = nh.subscribe("/fiducial_vertices", 1, &ArucoImagesTest::vertices_callback, this);
    got_vertices = false;
  }

  virtual void TearDown() { delete it;}

  void publish_image(std::string file) {
    cv::Mat image = cv::imread(image_directory+file, CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    c_info.header.stamp = ros::Time::now();
    CameraInfoPub.publish(c_info);
  }

  void vertices_callback(const fiducial_msgs::FiducialArray& f) {
    got_vertices = true;
    fiducials = f;
  }

  ros::NodeHandle nh;

  // Set up Publishing of static images
  image_transport::ImageTransport* it;
  image_transport::Publisher image_pub;

  sensor_msgs::CameraInfo c_info;
  ros::Publisher CameraInfoPub;

  std::string image_directory;

  // Set up subscribing
  bool got_vertices;
  fiducial_msgs::FiducialArray fiducials;
  ros::Subscriber vertices_sub;
};

TEST_F(ArucoImagesTest, tag_01_d7_14cm) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_vertices) {
    publish_image("tag_01_d7_14cm.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_LE(1, fiducials.fiducials.size());

  const fiducial_msgs::Fiducial& vertices = fiducials.fiducials[0];
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

int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ArucoImagesTest");
  return RUN_ALL_TESTS();
}
