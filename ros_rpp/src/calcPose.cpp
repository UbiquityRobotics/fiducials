
/*

This node listens for Fiducial messages and computes the pose of
the fiducial relative to a fiducial centered on the origin.

*/

#include <stdio.h>
#include <math.h>
#include <malloc.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/CameraInfo.h>
#include <fiducials_ros/Fiducial.h>
#include <ros_rpp/FiducialTransform.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "RPP.h"


class RosRpp {
  cv::Mat model;
  cv::Mat ipts;
  cv::Mat K;
  cv::Mat dist;
  
  bool haveCamInfo;
  double fiducialLen;
  int currentFrame;
  ros::Time frameTime;
  std::map<int, tf::Transform> frameTransforms;
  ros::Publisher tfPub;
  ros::Subscriber verticesSub;
  ros::Subscriber camInfoSub;

public:
  RosRpp(ros::NodeHandle);
  void fiducialCallback(const fiducials_ros::Fiducial::ConstPtr& msg);
  void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void undistortPoints(cv::Mat pts);
};

// Radians to degrees
double r2d(double r) 
{
  return r / M_PI * 180.0;
}

// Euclidean distance between two points
double dist(cv::Mat pts, int i1, int i2)
{
  double x1 = pts.at<double>(0, i1);
  double y1 = pts.at<double>(1, i1);
  double x2 = pts.at<double>(0, i2);
  double y2 = pts.at<double>(1, i2);

  double dx = x1 - x2;
  double dy = y1 - y2;

  return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
double calcFiducialArea(cv::Mat pts)
{
  double a1 = dist(pts, 0, 1);
  double b1 = dist(pts, 0, 3);
  double c1 = dist(pts, 1, 3);
  
  double a2 = dist(pts, 1, 2);
  double b2 = dist(pts, 2, 3);
  double c2 = c1;

  double s1 = (a1 + b1 + c1) / 2.0;
  double s2 = (a2 + b2 + c2) / 2.0;

  a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
  a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
  return a1+a2;
}


void RosRpp::undistortPoints(cv::Mat pts)
{
  cv::Mat src(1, pts.cols, CV_64FC2);
  cv::Mat dst(1, pts.cols, CV_64FC2);

  for (int i=0; i<pts.cols; i++) {
    src.at<cv::Vec2d>(0, i)[0] = pts.at<double>(0, i);
    src.at<cv::Vec2d>(0, i)[1] = pts.at<double>(1, i);
  }

  cv::vector<cv::Point2f> dest;
  cv::undistortPoints(src, dst, K, dist);

  for (int i=0; i<pts.cols; i++) {
    pts.at<double>(0, i) = dst.at<cv::Vec2d>(0, i)[0];
    pts.at<double>(1, i) = dst.at<cv::Vec2d>(0, i)[1];
  } 
}


void RosRpp::fiducialCallback(const fiducials_ros::Fiducial::ConstPtr& msg)
{
  ROS_INFO("date %d id %d direction %d", 
	   msg->header.stamp.sec, msg->fiducial_id,
	   msg->direction);

  if (!haveCamInfo) {
    ROS_ERROR("No camera info");
    return;
  }

  if (currentFrame != msg->image_seq) {
    //frameTime = msg->header.stamp;
    frameTime = ros::Time::now();
    currentFrame = msg->image_seq;
    }

  /* The verices are ordered anti-clockwise, starting with the top-left
     we want to end up with this:
       2  3
       1  0
  */

  printf("frame %d fid %d dir %d vertices %lf %lf %lf %lf %lf %lf %lf %lf\n",
	 msg->image_seq, msg->fiducial_id, msg->direction, 
	 msg->x0, msg->y0, msg->x1, msg->y1,
	 msg->x2, msg->y2, msg->x3, msg->y3);
  
  switch(msg->direction) {
    case 0: 
      // 0 1 2 3
      ipts.at<double>(0,0) = msg->x0;
      ipts.at<double>(1,0) = msg->y0;
      ipts.at<double>(0,1) = msg->x1;
      ipts.at<double>(1,1) = msg->y1;
      ipts.at<double>(0,2) = msg->x2;
      ipts.at<double>(1,2) = msg->y2;
      ipts.at<double>(0,3) = msg->x3;
      ipts.at<double>(1,3) = msg->y3;
      break;
       
    case 1: 
      // 3 0 1 2
      ipts.at<double>(0,0) = msg->x3;
      ipts.at<double>(1,0) = msg->y3;
      ipts.at<double>(0,1) = msg->x0;
      ipts.at<double>(1,1) = msg->y0;
      ipts.at<double>(0,2) = msg->x1;
      ipts.at<double>(1,2) = msg->y1;
      ipts.at<double>(0,3) = msg->x2;
      ipts.at<double>(1,3) = msg->y2;
        break;
	
    case 2:
      // 2 3 0 1
      ipts.at<double>(0,0) = msg->x2;
      ipts.at<double>(1,0) = msg->y2;
      ipts.at<double>(0,1) = msg->x3;
      ipts.at<double>(1,1) = msg->y3;
      ipts.at<double>(0,2) = msg->x0;
      ipts.at<double>(1,2) = msg->y0;
      ipts.at<double>(0,3) = msg->x1;
      ipts.at<double>(1,3) = msg->y1;
      break;
      
    case 3: 
      // 1 2 3 0
      ipts.at<double>(0,0) = msg->x1;
      ipts.at<double>(1,0) = msg->y1;
      ipts.at<double>(0,1) = msg->x2;
      ipts.at<double>(1,1) = msg->y2;
      ipts.at<double>(0,2) = msg->x3;
      ipts.at<double>(1,2) = msg->y3;
      ipts.at<double>(0,3) = msg->x0;
      ipts.at<double>(1,3) = msg->y0;
      break;
  }

  undistortPoints(ipts);
    
  cv::Mat rotation;
  cv::Mat translation;
  int iterations;
  double obj_err;
  double img_err;

  if(!RPP::Rpp(model, ipts, rotation, translation, iterations, obj_err, img_err)) {
    ROS_ERROR("Cannot find transform for fiducial %d", msg->fiducial_id);
    return;
  }
    
  ROS_INFO("fid %d iterations %d object error %f image error %f", 
	   msg->fiducial_id, iterations, obj_err, img_err);
    
  tf::Matrix3x3 m1(rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2),
		   rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2),
		   rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2));

  tf::Vector3 t1(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));

  tf::Transform trans1(m1, t1);

  frameTransforms[msg->fiducial_id] = trans1;
  t1 = trans1.getOrigin();
  m1 = trans1.getBasis();

  double r, p, y;
  m1.getRPY(r, p, y);
  ROS_INFO("fid %d T: %.3lf %.3lf %.3lf R: %.2f %.2f %.2f",
	   msg->fiducial_id,
	   t1.x(), t1.y(), t1.z(),
	   r2d(r), r2d(p), r2d(y));
  
  ros_rpp::FiducialTransform ft;
  geometry_msgs::Transform transform;
  ft.header.stamp = frameTime;
  char t1name[32];
  sprintf(t1name, "fiducial_%d", msg->fiducial_id);
  ft.header.frame_id = t1name;

  transform.translation.x = t1.x();
  transform.translation.y = t1.y();		
  transform.translation.z = t1.z(); 
  tf::Quaternion q = trans1.getRotation();
  transform.rotation.w = q.w();
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  ft.transform = transform;
  ft.fiducial_id = msg->fiducial_id;
  ft.image_seq = msg->image_seq;
  ft.image_error = img_err;
  ft.object_error = obj_err;
  ft.fiducial_area = calcFiducialArea(ipts);
  tfPub.publish(ft);
}


void RosRpp::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (haveCamInfo) {
    return;
  }

  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      K.at<double>(i, j) = msg->K[i*3+j];
    }
  }
  
  printf("camera intrinsics:\n");
  RPP::Print(K);

  for (int i=0; i<5; i++) {
    dist.at<double>(0,i) = msg->D[i];
  }
  
  printf("distortion coefficients:\n");
  RPP::Print(dist);

  haveCamInfo = true;
}


RosRpp::RosRpp(ros::NodeHandle nh)
{
  // Camera intrinsics
  K = cv::Mat::zeros(3, 3, CV_64F);

  // distortion coefficients
  dist = cv::Mat::zeros(1, 5, CV_64F);

  // homogeneous 2D points
  ipts = cv::Mat::ones(3, 4, CV_64F);

  // 3D points of a fiducial at the origin z = 0
  model = cv::Mat::zeros(3, 4, CV_64F); 

  haveCamInfo = false;

  camInfoSub = nh.subscribe("/camera_info",
			    1,
			    &RosRpp::camInfoCallback,
			    this);
  
  nh.param<double>("fiducial_len", fiducialLen, 0.146);

  /*
     Vertex ordering:

       2  3
       1  0

     World 
     y
     ^
     |
     `-->x

    Fiducial with origin at center:
  */

  model.at<double>(0,0) = -fiducialLen / 2.0;
  model.at<double>(1,0) = -fiducialLen / 2.0;
  
  model.at<double>(0,1) =  fiducialLen / 2.0;
  model.at<double>(1,1) = -fiducialLen / 2.0;

  model.at<double>(0,2) =  fiducialLen / 2.0;
  model.at<double>(1,2) =  fiducialLen / 2.0;

  model.at<double>(0,3) = -fiducialLen / 2.0;
  model.at<double>(1,3) =  fiducialLen / 2.0;

  currentFrame = 0;

  tfPub = nh.advertise<ros_rpp::FiducialTransform>("fiducial_transforms", 3);

  verticesSub = nh.subscribe("vertices",
			     1,
			     &RosRpp::fiducialCallback,
			     this);
} 


int main(int argc, char *argv[]) 
{
  ros::init(argc, argv, "rpp_pose");
  ros::NodeHandle node;
  RosRpp RosRpp(node);
  ROS_INFO("rpp_pose started");

  ros::spin();
}

