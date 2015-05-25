
#include "fiducial_pose/Fiducial.h"
#include "fiducial_pose/FiducialTransform.h"
#include <sensor_msgs/CameraInfo.h>

class RosRpp {
  cv::Mat model;
  cv::Mat ipts;
  cv::Mat K;
  cv::Mat dist;
  
  bool haveCamInfo;
  int currentFrame;
  ros::Time frameTime;
  std::map<int, tf::Transform> frameTransforms;
  ros::Publisher tfPub;
  ros::Subscriber verticesSub;
  ros::Subscriber camInfoSub;

  bool doUndistort;
  double fiducialLen;

public:
  RosRpp(double fiducialLen, bool doUndistort);

  void fiducialCallback(fiducial_pose::Fiducial* fiducial,
			fiducial_pose::FiducialTransform* transform);

  void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};
