
/*

This library computes the pose of
the fiducial relative to a fiducial centered on the origin.

*/

#include <stdio.h>
#include <math.h>
#include <malloc.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "RPP.h"
#include "fiducial_pose/rosrpp.h"

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


void undistortPoints(cv::Mat pts, cv::Mat K, cv::Mat dist, bool doUndistort)
{
  if (!doUndistort) {
    printf("fx %lf fy %lf cx %lf cy %lf", K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2));
    for (int i=0; i<pts.cols; i++) {
      pts.at<double>(0, i) = (pts.at<double>(0, i) - K.at<double>(0, 2)) / K.at<double>(0, 0);
      pts.at<double>(1, i) = (pts.at<double>(1, i) - K.at<double>(1, 2)) / K.at<double>(1, 1);
    }
  }
  else {
    cv::Mat src(1, pts.cols, CV_64FC2);
    cv::Mat dst(1, pts.cols, CV_64FC2);
  
    for (int i=0; i<pts.cols; i++) {
      src.at<cv::Vec2d>(0, i)[0] = pts.at<double>(0, i);
      src.at<cv::Vec2d>(0, i)[1] = pts.at<double>(1, i);
    }
    std::vector<cv::Point2f> dest;
    cv::undistortPoints(src, dst, K, dist);

    for (int i=0; i<pts.cols; i++) {
      pts.at<double>(0, i) = dst.at<cv::Vec2d>(0, i)[0];
      pts.at<double>(1, i) = dst.at<cv::Vec2d>(0, i)[1];
    } 
  }
}


bool RosRpp::fiducialCallback(fiducial_pose::Fiducial* msg,
		       	      fiducial_pose::FiducialTransform* ft)
{
  ROS_INFO("id %d direction %d", msg->fiducial_id, msg->direction);

  if (!haveCamInfo) {
    ROS_ERROR("No camera info");
    return false;
  }

  /* The verices are ordered anti-clockwise, starting with the top-left
     we want to end up with this:
       2  3
       1  0
  */

  printf("fid %d dir %d vertices %lf %lf %lf %lf %lf %lf %lf %lf\n",
	 msg->fiducial_id, msg->direction, 
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

#if 0
  // This was an attempt to replace RPP with OpenCV's pose estimation.
  // It seemed to be more noisy, probably because it is not designed to work with coplanar points.

  std::vector<cv::Point2f> imagePoints;
  std::vector<cv::Point3f> model2;
  for (int i=0; i<4; i++) {
      imagePoints.push_back(cv::Point2f(ipts.at<double>(0, i), ipts.at<double>(1, i)));
      model2.push_back(cv::Point3f(model.at<double>(0, i), model.at<double>(1, i), model.at<double>(2, i)));
      ROS_INFO("model %f %f %f", model.at<double>(0, i), model.at<double>(1, i), model.at<double>(2, i));
  }

  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat tvec(3,1,cv::DataType<double>::type);

  cv::solvePnP(model2, imagePoints, K, dist, rvec, tvec, false, CV_EPNP);

  ROS_INFO("tvec %lf %lf %lf", tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
  ROS_INFO("rvec %lf %lf %lf", r2d(rvec.at<double>(0, 0)), r2d(rvec.at<double>(1, 0)), r2d(rvec.at<double>(2, 0)));

  tf::Vector3 translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
  tf::Quaternion rotation;
  rotation.setRPY(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
  tf::Transform transform(rotation, translation);

  ft->transform.translation.x = translation.x();
  ft->transform.translation.y = translation.y();		
  ft->transform.translation.z = translation.z(); 

  ft->transform.rotation.w = rotation.w();
  ft->transform.rotation.x = rotation.x();
  ft->transform.rotation.y = rotation.y();
  ft->transform.rotation.z = rotation.z();
  ft->fiducial_id = msg->fiducial_id;
#else

  undistortPoints(ipts, K, dist, doUndistort);
    
  cv::Mat rotation;
  cv::Mat translation;
  int iterations;
  double obj_err;
  double img_err;

  std::vector<RPP::Solution> sol;

  std::map<int, cv::Mat>::iterator it;

  it = prevRots.find(msg->fiducial_id);
  if (it == prevRots.end()) 
     rotation = cv::Mat();
  else 
     rotation = it->second;

  if(!RPP::Rpp(model, ipts, rotation, translation, iterations, obj_err, img_err, sol)) {
    ROS_ERROR("Cannot find transform for fiducial %d", msg->fiducial_id);
    return false;
  }
    
  prevRots[msg->fiducial_id] = rotation;

  //ROS_INFO("fid %d iterations %d object error %f image error %f", 
  //	   msg->fiducial_id, iterations, obj_err, img_err);
    
  /*
  ROS_INFO("fid %d solutions %lu", msg->fiducial_id, sol.size());
  for (unsigned int i=0; i<sol.size(); i++) {
    double r, p, y;
    tf::Matrix3x3 m(sol[i].R.at<double>(0,0), sol[i].R.at<double>(0,1), sol[i].R.at<double>(0,2),
		    sol[i].R.at<double>(1,0), sol[i].R.at<double>(1,1), sol[i].R.at<double>(1,2),
		    sol[i].R.at<double>(2,0), sol[i].R.at<double>(2,1), sol[i].R.at<double>(2,2));

    tf::Vector3 t(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));    
    
    tf::Transform trans(m, t);

    for (int j=0; j<4; j++) {
      tf::Vector3 pi(model.at<double>(0,j), model.at<double>(1,j), 0);
      tf::Vector3 pw = trans * pi;
      ROS_INFO("vertex %d %f %f %f", j, pw.x(), pw.y(), pw.z());
    }
    m.getRPY(r, p, y);
    ROS_INFO("fid %d sol %d R: %.2f %.2f %.2f", msg->fiducial_id, i, r2d(r), r2d(p), r2d(y));
  }
  */

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
  
  /*
  ft->header.stamp = frameTime;
  char t1name[32];
  sprintf(t1name, "fiducial_%d", msg->fiducial_id);
  ft->header.frame_id = t1name;
  */

  ft->transform.translation.x = t1.x();
  ft->transform.translation.y = t1.y();		
  ft->transform.translation.z = t1.z(); 
  tf::Quaternion q = trans1.getRotation();
  ft->transform.rotation.w = q.w();
  ft->transform.rotation.x = q.x();
  ft->transform.rotation.y = q.y();
  ft->transform.rotation.z = q.z();
  
  ft->fiducial_id = msg->fiducial_id;
  //  ft->image_seq = msg->image_seq;
  ft->image_error = img_err;
  ft->object_error = obj_err;
  ft->fiducial_area = calcFiducialArea(ipts);
#endif
  return true;
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


RosRpp::RosRpp(double fiducialLen, bool doUndistort)
{
  this->fiducialLen = fiducialLen;
  this->doUndistort = doUndistort;

  // Camera intrinsics
  K = cv::Mat::zeros(3, 3, CV_64F);

  // distortion coefficients
  dist = cv::Mat::zeros(1, 5, CV_64F);

  // homogeneous 2D points
  ipts = cv::Mat::ones(3, 4, CV_64F);

  // 3D points of a fiducial at the origin z = 0
  model = cv::Mat::zeros(3, 4, CV_64F); 

  haveCamInfo = false;

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

  model.at<double>(0,0) =  fiducialLen / 2.0;
  model.at<double>(1,0) = -fiducialLen / 2.0;
  
  model.at<double>(0,1) = -fiducialLen / 2.0;
  model.at<double>(1,1) = -fiducialLen / 2.0;

  model.at<double>(0,2) = -fiducialLen / 2.0;
  model.at<double>(1,2) =  fiducialLen / 2.0;

  model.at<double>(0,3) =  fiducialLen / 2.0;
  model.at<double>(1,3) =  fiducialLen / 2.0;

  currentFrame = 0;
} 
