#include <gtest/gtest.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fiducial_pose/Fiducial.h>
#include <fiducial_pose/FiducialTransformArray.h>

class FiducialsVerticesTests : public ::testing::Test {
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

    vertices_sub = nh.subscribe("/fiducial_vertices", 1, &FiducialsVerticesTests::vertices_callback, this);
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

  void vertices_callback(const fiducial_pose::Fiducial& f) {
    got_vertices = true;
    vertices = f;
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
  fiducial_pose::Fiducial vertices;
  ros::Subscriber vertices_sub;
};

TEST_F(FiducialsVerticesTests, fiducial_30) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_vertices) {
    publish_image("fiducial_30.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(30, vertices.fiducial_id);

  ASSERT_DOUBLE_EQ(788.7144775390625000, vertices.x0);
  ASSERT_DOUBLE_EQ(424.7263488769531250, vertices.y0);
  ASSERT_DOUBLE_EQ(797.6848144531250000, vertices.x1);
  ASSERT_DOUBLE_EQ(664.1170654296875000, vertices.y1);
  ASSERT_DOUBLE_EQ(552.2851562500000000, vertices.x2);
  ASSERT_DOUBLE_EQ(668.5111083984375000, vertices.y2);
  ASSERT_DOUBLE_EQ(551.7211303710937500, vertices.x3);
  ASSERT_DOUBLE_EQ(426.8039855957031250, vertices.y3);
}

TEST_F(FiducialsVerticesTests, fiducial_34) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_vertices) {
    publish_image("fiducial_34.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(34, vertices.fiducial_id);

  ASSERT_DOUBLE_EQ(840.3661499023437500, vertices.x0);
  ASSERT_DOUBLE_EQ(105.3488464355468750, vertices.y0);
  ASSERT_DOUBLE_EQ(839.5590820312500000, vertices.x1);
  ASSERT_DOUBLE_EQ(456.3438720703125000, vertices.y1);
  ASSERT_DOUBLE_EQ(506.3190307617187500, vertices.x2);
  ASSERT_DOUBLE_EQ(462.5921936035156250, vertices.y2);
  ASSERT_DOUBLE_EQ(493.5599365234375000, vertices.x3);
  ASSERT_DOUBLE_EQ(118.8109970092773437, vertices.y3);
}

TEST_F(FiducialsVerticesTests, fiducial_35) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_vertices) {
    publish_image("fiducial_35.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(35, vertices.fiducial_id);

  ASSERT_DOUBLE_EQ(863.0485839843750000, vertices.x0);
  ASSERT_DOUBLE_EQ(120.1449050903320312, vertices.y0);
  ASSERT_DOUBLE_EQ(861.0425415039062500, vertices.x1);
  ASSERT_DOUBLE_EQ(459.1877746582031250, vertices.y1);
  ASSERT_DOUBLE_EQ(532.6334838867187500, vertices.x2);
  ASSERT_DOUBLE_EQ(463.8345336914062500, vertices.y2);
  ASSERT_DOUBLE_EQ(524.2216186523437500, vertices.x3);
  ASSERT_DOUBLE_EQ(126.8642807006835937, vertices.y3);
}

int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FiducialsVerticesTests");
  return RUN_ALL_TESTS();
}