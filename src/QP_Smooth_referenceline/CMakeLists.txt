cmake_minimum_required(VERSION 3.0.2)
project(QP_Smooth_referenceline)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
set(CMAKE_CXX_STANDARD 11)
file(GLOB_RECURSE PYTHON2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
set(PYTHON2.7_INLCUDE_DIRS "/usr/include/python2.7")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES QP_Smooth_referenceline
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(./eigen-3.4.0)

link_directories(
  /usr/local/lib 
)

include_directories(
      include
      ${catkin_INCLUDE_DIRS}
      ${PYTHON2.7_INLCUDE_DIRS}
)
add_executable(osqp1  src/OSQP.cpp
                      src/main.cpp
)
target_link_libraries(osqp1 ${PYTHON2.7_LIB}
                            libosqp.so  
                            libOsqpEigen.so 
                            libqdldl.so)

