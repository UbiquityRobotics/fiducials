
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
#include <fiducials_ros/Fiducial.h>

#include <opencv2/core/core.hpp>
#include "RPP.h"


//TODO: have this as params
double fiducialLen = 0.146; // in meters
//double fiducialLen = 0.2055; // At the dojo measured 11/17/14

/*
From camera info - TODO: load camera_info file

image_width: 640
image_height: 480
camera_name: pgr_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [1019.42121729448, 0, 362.120173989305,
         0, 1018.80123863486, 280.729668298301,
         0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.385773196965028, 0.379304602782537, 0.000508189783669918, 0.0016178175839165, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 
         0, 1, 0,
         0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [978.116943359375, 0, 365.075073083302, 0,
         0, 992.935852050781, 281.893622991109, 0,
         0, 0, 1, 0]

camera_matrix:
  rows: 3
  cols: 3
  data: 1019.42121729448, 0, 362.120173989305, 
         0, 1018.80123863486, 280.729668298301, 
         0, 0, 1]

*/
double fx = 1019.42121729448;
double fy = 1018.80123863486;
double cx = 362.120173989305;
double cy = 280.729668298301;

double kc[] = {-0.385773196965028, 0.379304602782537, 0.000508189783669918, 0.0016178175839165, 0};



class Fiducial {
public:
  double vertices[12];
  int id;

  Fiducial() {};
};

class Localization {
  cv::Mat model;
  cv::Mat ipts;
  cv::Mat K;

  int currentFrame;
  ros::Time frameTime;
  std::map<int, tf::Transform> frameTransforms;
  tf::TransformBroadcaster tf_pub;
  ros::Subscriber sub;

public:
  Localization(ros::NodeHandle);
  void fiducialCallback(const fiducials_ros::Fiducial::ConstPtr& msg);
};

#define MAX_POINTS 40

double r2d(double r) 
{
  return r / M_PI * 180.0;
}

void ideal2Observe(cv::Mat pts)
{
  for (int i=0; i<pts.cols; i++) {
    //printf("before %f %f\n", pts.at<double>(0, i), pts.at<double>(1, i));
    const double xc = (pts.at<double>(0, i) - cx) / fx;
    const double yc = (pts.at<double>(1, i) - cy) / fy;
    
#if 0
    const double r2 = (xc*xc) + (yc*yc);
    const double r4 = r2*r2;
    const double r6 = r4*r2;
    const double cdist = 1 + kc[0] * r2 + kc[1] * r4 + kc[4] * r6;
    
    const double a1 = 2*xc*yc;
    const double a2 = r2 + 2*(xc*xc);
    const double a3 = r2 + 2*(yc*yc);
    
    pts.at<double>(0, i) = (xc * cdist) + (kc[2]*a1 + kc[3]*a2);
    pts.at<double>(1, i) = (yc * cdist) + (kc[2]*a3 + kc[3]*a1);   
#else
    pts.at<double>(0, i) = xc;
    pts.at<double>(1, i) = yc;
#endif

    //printf("after %f %f\n", pts.at<double>(0, i), pts.at<double>(1, i));
  }
}

/*******************************************************************************************************/

void Localization::fiducialCallback(const fiducials_ros::Fiducial::ConstPtr& msg)
{
    ROS_INFO("date %d id %d direction %d", 
	     msg->header.stamp.sec, msg->fiducial_id,
	     msg->direction);

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
      
    default:
      return;
    }

    ideal2Observe(ipts);
  
    cv::Mat rotation;
    cv::Mat translation;
    int iterations;
    double obj_err;
    double img_err;

    /*
    printf("K\n");
    RPP::Print(K);
    printf("ipts\n");
    RPP::Print(ipts);
    printf("ipts2\n");
    ipts = K * ipts;
    RPP::Print(ipts);
    */
    if(!RPP::Rpp(model, ipts, rotation, translation, iterations, obj_err, img_err)) {
        fprintf(stderr, "Error with RPP\n");
	return;
    }
    
    /*
    RPP::Print(translation);
    RPP::Print(rotation);
    */

    printf("Number of iterations: %d\n", iterations);
    printf("Object error: %f\n", obj_err);
    printf("Image error: %f\n", img_err);

    
    tf::Matrix3x3 m1(rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2),
		     rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2),
		     rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2));

    tf::Vector3 t1(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));

    tf::Transform trans1(m1, t1);
    //trans1 = trans1.inverse();
    frameTransforms[msg->fiducial_id] = trans1;
    t1 = trans1.getOrigin();
    m1 = trans1.getBasis();

    double r, p, y;
    m1.getRPY(r, p, y);
    ROS_INFO("fid %d T: %.3lf %.3lf %.3lf R: %.2f %.2f %.2f",
             msg->fiducial_id,
             t1.x(), t1.y(), t1.z(),
             r2d(r), r2d(p), r2d(y));
    
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = frameTime;
    char t1name[32];
    sprintf(t1name, "fiducial_%d", msg->fiducial_id);
    transform.header.frame_id = "camera";
    transform.child_frame_id = t1name;

    transform.transform.translation.x = t1.x();
    transform.transform.translation.y = t1.y();		
    transform.transform.translation.z = t1.z(); 
    tf::Quaternion q = trans1.getRotation();
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    tf_pub.sendTransform(transform);
}

Localization::Localization(ros::NodeHandle nh)
{
    sub = nh.subscribe("/fiducials_localization/vertices",
                       1,
                       &Localization::fiducialCallback,
                       this);

    // Camera intrinsics
    K = cv::Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;
    K.at<double>(2,2) = 1;
    
    // homogeneous 2D points
    ipts = cv::Mat::ones(3, 4, CV_64F);

    // 3D points of a fiducial at the origin z = 0
    model = cv::Mat::zeros(3, 4, CV_64F); 

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

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "rpp_pose");
    ros::NodeHandle node;
    Localization localization(node);
    ROS_INFO("rpp_pose started");

    ros::spin();
}
