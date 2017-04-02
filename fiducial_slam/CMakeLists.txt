cmake_minimum_required(VERSION 2.8.3)
project(fiducial_slam)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   nav_msgs
   tf2_msgs
   geometry_msgs
   sensor_msgs
   visualization_msgs
   message_generation
)

catkin_python_setup()

##############
## Messages ##
##############

add_message_files(
   FILES
   FiducialMapEntry.msg
   FiducialMapEntryArray.msg
)

##############
## Services ##
##############

add_service_files(
  FILES
  InitializeMap.srv
)

generate_messages()
catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/move_origin.py scripts/init_map.py
        nodes/init_amcl.py nodes/fiducial_slam_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Tests ##
###########


if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED COMPONENTS 
        tf2_ros)

        find_package(catkin REQUIRED COMPONENTS
          roscpp
          cv_bridge
          image_transport)

        find_package(OpenCV REQUIRED)

        # We are not using the big bag test right now

# These tests need to be moved to aruco_detect

        # catkin_download_test_data(${PROJECT_NAME}_ceilingline.bag
        #                           http://www.fxpal.com/robots/ceilingline.bag
        #                           DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
        #                           MD5 05b69d64ab02d79ec4fa4be3a7de05af) 

        #add_rostest(test/create_map.xml)
        #add_rostest(test/init_map.xml)
        # add_rostest(test/ceiling_test_2.xml)

        #include_directories(${catkin_INCLUDE_DIRS})

        # Tests need c++11
        #add_definitions(-std=c++11)
        #add_rostest_gtest(localization_test 
        #  test/localization.test 
        #  test/localization_test.cpp)
        #target_link_libraries(localization_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
