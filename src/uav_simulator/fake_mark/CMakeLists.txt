cmake_minimum_required(VERSION 2.8.3)
project(mark_odom)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  quadrotor_msgs
  traj_utils
  traj_opt
  plan_env
  cv_bridge
  relative_loc
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (mark_odom src/mark_odom.cpp )
target_link_libraries(mark_odom
  ${catkin_LIBRARIES})
