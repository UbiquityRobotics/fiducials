
//#include <ros/ros.h>

#include "fiducials_rviz.h"

int main( int argc, char** argv )
{
  //ros::init(argc, argv, "basic_shapes");
  //ros::NodeHandle n;
  //ros::Rate r(1);

  void* rd = initRviz(argc, argv, "test");

  //while (ros::ok())
  while (1) 
  {
    sendMarker(rd, "fiducial_frame", 0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1);

    //r.sleep();
  }
}
