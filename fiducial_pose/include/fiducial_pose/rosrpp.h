
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialTransform.h"
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

  std::map<int, cv::Mat>prevRots;

public:
  RosRpp(double fiducialLen, bool doUndistort);

  bool fiducialCallback(fiducial_msgs::Fiducial* fiducial,
			fiducial_msgs::FiducialTransform* transform);

  void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};
