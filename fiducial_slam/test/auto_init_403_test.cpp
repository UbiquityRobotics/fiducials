/*
Test to verify that when the map is initialized from
a single fiducial that the robot's pose is identity
and the map entry for the fiducial is reasonable
*/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fiducial_msgs/FiducialMapEntry.h>
#include <fiducial_msgs/FiducialMapEntryArray.h>


class AutoInitTest : public ::testing::Test {
protected:
  virtual void SetUp() { 
    it = new image_transport::ImageTransport(nh);
    image_pub = it->advertise("camera/image", 1);

    CameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 5);

    c_info.header.frame_id = "camera";
    c_info.height = 960;
    c_info.width = 1280;
    c_info.distortion_model = "plumb_bob";

    double data_D[] = {0.1349735087283542, -0.2335869827451621,
                       0.0006697030315075139, 0.004846737465872353, 0.0};

    c_info.D = std::vector<double> (data_D,
                                    data_D + sizeof(data_D)/ sizeof(double));
   
    c_info.K = {1006.126285753055, 0.0,               655.8639244150409, 
                0.0,               1004.015433012594, 490.6140221242933,
                0.0,               0.0,               1.0};

    c_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    c_info.P = {1021.54345703125, 0.0,            661.9091982335958, 0.0,
                0.0,              1025.251953125, 490.6380671707448, 0.0,
                0.0,              0.0,            1.0,               0.0};

    ros::NodeHandle nh_priv("~");
    nh_priv.getParam("image_directory", image_directory);

    pose_sub = nh.subscribe("/fiducial_pose", 1, 
                            &AutoInitTest::pose_callback, this);
    got_pose = false;

    fiducial_msgs::FiducialMapEntryArray map;

    map_sub = nh.subscribe("/fiducial_map", 1, 
                            &AutoInitTest::map_callback, this);
    got_map = false;
  }

  virtual void TearDown() { delete it;}

  void publish_image(std::string file)
  {
    cv::Mat image = cv::imread(image_directory+file, cv::IMREAD_COLOR);
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                   "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    c_info.header.stamp = ros::Time::now();
    CameraInfoPub.publish(c_info);
  }

  void map_callback(const fiducial_msgs::FiducialMapEntryArray& msg)
  {
    got_map = true;
    map = msg;
  }

  void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    got_pose = true;
    pose = msg;
  }

  ros::NodeHandle nh;

  // Set up Publishing of static images
  image_transport::ImageTransport* it;
  image_transport::Publisher image_pub;

  sensor_msgs::CameraInfo c_info;
  ros::Publisher CameraInfoPub;

  std::string image_directory;

  ros::Subscriber map_sub;
  ros::Subscriber pose_sub;

  bool got_map;
  bool got_pose;

  fiducial_msgs::FiducialMapEntryArray map;
  geometry_msgs::PoseWithCovarianceStamped pose;

};

TEST_F(AutoInitTest, tag_403_d7_14cm) {
  ros::Rate loop_rate(5);
  while (nh.ok() && (!got_pose || !got_map)) {
    publish_image("403.jpg");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_NEAR(0, pose.pose.pose.position.x, 0.001);
  ASSERT_NEAR(0, pose.pose.pose.position.y, 0.001);
  ASSERT_NEAR(0, pose.pose.pose.position.z, 0.001);

  ASSERT_NEAR(1, pose.pose.pose.orientation.w, 0.001);
  ASSERT_NEAR(0, pose.pose.pose.orientation.x, 0.001);
  ASSERT_NEAR(0, pose.pose.pose.orientation.y, 0.001);
  ASSERT_NEAR(0, pose.pose.pose.orientation.z, 0.001);

  ASSERT_LE(1, map.fiducials.size());
  
  const fiducial_msgs::FiducialMapEntry &fid = map.fiducials[0];
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
  ros::init(argc, argv, "AutoInitTest");
  return RUN_ALL_TESTS();
}
