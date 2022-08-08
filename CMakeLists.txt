cmake_minimum_required(VERSION 3.0.2)
project(slam_evaluation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf
 DEPENDS EIGEN3
)

add_executable(slam_evaluation src/slam_evaluation.cpp)
target_link_libraries(slam_evaluation ${catkin_LIBRARIES} Eigen3::Eigen)