cmake_minimum_required(VERSION 2.8.3)
project(fiducial_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   std_msgs
   nav_msgs
   tf2_msgs
   geometry_msgs
   sensor_msgs
   visualization_msgs)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/move_origin.py scripts/init_map.py
        nodes/init_amcl.py nodes/fiducial_slam.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


###########
## Tests ##
###########


if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED
                     COMPONENTS tf2_ros)

        catkin_download_test_data(${PROJECT_NAME}_ceilingline.bag
                                  http://www.fxpal.com/robots/ceilingline.bag
                                  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
                                  MD5 05b69d64ab02d79ec4fa4be3a7de05af) 

        add_rostest(test/create_map.xml)
        add_rostest(test/localization_test.xml)

        add_rostest(test/ceiling_test_2.xml)
endif()

