cmake_minimum_required(VERSION 2.8.3)
project(online_calibration)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  	     message_generation
  	     geometry_msgs
  	     nav_msgs
  	     sensor_msgs
  	     roscpp
  	     rospy
  	     std_msgs
  	     tf
  	     tf_conversions
  	     rosbag
  	     livox_ros_driver
  	     eigen_conversions
  	     pcl_conversions
  	     pcl_ros
  	     message_filters
  	     cv_bridge
  	     std_srvs)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)



include_directories(include
        	    ${catkin_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
		    ${PCL_INCLUDE_DIRS}
		    ${OpenCV_INCLUDE_DIRS}
        	    ${SUITESPARSE_INCLUDE_DIRS})

##################
## ROS messages ##
##################

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime 
  	       pcl_conversions pcl_ros message_filters std_srvs tf_conversions tf 
	       eigen_conversions DEPENDS PCL OpenCV INCLUDE_DIRS)

add_executable (online_calibration 
		src/onlineCalib.cpp
		)
add_executable (online_display 
		src/onlineDisplay.cpp
		)

target_link_libraries(online_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(online_display ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})



