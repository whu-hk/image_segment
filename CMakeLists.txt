cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)

project(image_segment)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
	roslib
    std_msgs
	sensor_msgs
	cv_bridge
	image_transport
	geometry_msgs
	visualization_msgs
	darknet_ros_msgs
	laser_segment
    message_generation
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   #Semantic_segment.msg
   #Semantic_segments.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs  laser_segment darknet_ros_msgs message_runtime 
  DEPENDS 
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) 
add_executable(image_segment_node src/image_segment.cpp)
target_link_libraries(image_segment_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${ARMADILLO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(image_segment_node image_segment_generate_messages_cpp) 
