cmake_minimum_required(VERSION 2.8.3)
project(nav_scripts)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nav_scripts
  CATKIN_DEPENDS
)

find_package(OpenCV REQUIRED)

