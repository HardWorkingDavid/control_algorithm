cmake_minimum_required(VERSION 3.0.2)
project(Purepursuit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


set(CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE PYTHON2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
set(PYTHON2.7_INLCUDE_DIRS "/usr/include/python2.7")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES huatu
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include
        ${PYTHON2.7_INLCUDE_DIRS}
)

add_executable(pp_controller     src/pure_pursuit.cpp
                              src/KinematicModel.cpp
                              src/main.cpp
                              src/pid_controller.cpp)
target_link_libraries(pp_controller ${PYTHON2.7_LIB})
