cmake_minimum_required(VERSION 2.8.3)
project(nav_scripts)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nav_scripts
  CATKIN_DEPENDS
)

find_package(OpenCV REQUIRED)

add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(nav_scripts src/gazebo_disparity_shim.cpp)

target_link_libraries(nav_scripts
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

add_executable(gazebo_disparity_shim src/shim_main.cpp)

target_link_libraries(gazebo_disparity_shim nav_scripts)
