cmake_minimum_required(VERSION 2.8.3)
project(victor)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

find_package(PCL REQUIRED)
find_package(OpenCV 2 REQUIRED)
set(CMAKE_BUILD_TYPE Debug)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
 LIBRARIES vision_nodelet
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs cv_bridge image_transport compressed_image_transport compressed_depth_image_transport tf_conversions nodelet roscpp_serialization
 DEPENDS PCL OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(vision_nodelet
  src/vision.cpp
  src/har_classifier.cpp
  src/color_cup_classifier.cpp
  src/circle_classifier.cpp
  src/gender_classifier.cpp
  src/sift_classifier.cpp
  src/person_tracker.cpp
)
add_dependencies(vision_nodelet
  ${PROJECT_NAME}_generate_messages
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
target_link_libraries(vision_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(vision_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
