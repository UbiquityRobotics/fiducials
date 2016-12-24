#include <gtest/gtest.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fiducial_pose/FiducialTransformArray.h>

class FiducialsTransformTests : public ::testing::Test {
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

    transforms_sub = nh.subscribe("/fiducial_transforms", 1, &FiducialsTransformTests::transforms_callback, this);
    got_transforms = false;
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

  void transforms_callback(const fiducial_pose::FiducialTransformArray& f) {
    got_transforms = true;
    transforms = f;
  }

  ros::NodeHandle nh;

  // Set up Publishing of static images
  image_transport::ImageTransport* it;
  image_transport::Publisher image_pub;

  sensor_msgs::CameraInfo c_info;
  ros::Publisher CameraInfoPub;

  std::string image_directory;

  // Set up subscribing
  bool got_transforms;
  fiducial_pose::FiducialTransformArray transforms;
  ros::Subscriber transforms_sub;
};

TEST_F(FiducialsTransformTests, fiducial_30) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_transforms) {
    publish_image("fiducial_30.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(30, transforms.transforms[0].fiducial_id);

  ASSERT_NEAR(0.0111652960709, transforms.transforms[0].transform.translation.x, 0.05);
  ASSERT_NEAR(0.0345085728175, transforms.transforms[0].transform.translation.y, 0.05);
  ASSERT_NEAR(0.6496505488590, transforms.transforms[0].transform.translation.z, 0.05);

  ASSERT_NEAR(0.9971363567790, transforms.transforms[0].transform.rotation.x, 0.05);
  ASSERT_NEAR(-0.007139744467, transforms.transforms[0].transform.rotation.y, 0.05);
  ASSERT_NEAR(0.0201383049329, transforms.transforms[0].transform.rotation.z, 0.05);
  ASSERT_NEAR(0.0725434953236, transforms.transforms[0].transform.rotation.w, 0.05);
}

TEST_F(FiducialsTransformTests, fiducial_34) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_transforms) {
    publish_image("fiducial_34.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(34, transforms.transforms[0].fiducial_id);

  ASSERT_NEAR(0.00557772645364, transforms.transforms[0].transform.translation.x, 0.05);
  ASSERT_NEAR(-0.0921505293117, transforms.transforms[0].transform.translation.y, 0.05);
  ASSERT_NEAR(0.459902136894, transforms.transforms[0].transform.translation.z, 0.05);

  ASSERT_NEAR(0.997516008884, transforms.transforms[0].transform.rotation.x, 0.05);
  ASSERT_NEAR(-0.00861731470528, transforms.transforms[0].transform.rotation.y, 0.05);
  ASSERT_NEAR(-0.0332893760892, transforms.transforms[0].transform.rotation.z, 0.05);
  ASSERT_NEAR(-0.0614765918635, transforms.transforms[0].transform.rotation.w, 0.05);
}

TEST_F(FiducialsTransformTests, fiducial_35) {
  ros::Rate loop_rate(5);
  while (nh.ok() && !got_transforms) {
    publish_image("fiducial_35.png");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(35, transforms.transforms[0].fiducial_id);

  ASSERT_NEAR(0.0181760973715, transforms.transforms[0].transform.translation.x, 0.05);
  ASSERT_NEAR(-0.0915359838771, transforms.transforms[0].transform.translation.y, 0.05);
  ASSERT_NEAR(0.470431107885, transforms.transforms[0].transform.translation.z, 0.05);

  ASSERT_NEAR(0.998738802658, transforms.transforms[0].transform.rotation.x, 0.05);
  ASSERT_NEAR(-0.00669542019471, transforms.transforms[0].transform.rotation.y, 0.05);
  ASSERT_NEAR(-0.00914052301956, transforms.transforms[0].transform.rotation.z, 0.05);
  ASSERT_NEAR(-0.0489124345408, transforms.transforms[0].transform.rotation.w, 0.05);
}

int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FiducialsTransformTests");
  return RUN_ALL_TESTS();
}