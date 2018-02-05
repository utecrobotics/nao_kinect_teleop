CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(nao_kinect_teleop)

# Operational Space Inverse Kinematics Control and RBDL packages
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(OSCR oscr REQUIRED)

## Find catkin macros and libraries
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospack
  roslib
)

CATKIN_PACKAGE()

INCLUDE_DIRECTORIES(
  include
  ${OSCR_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
  ${OSCR_LIBRARY_DIRS}
)

ADD_LIBRARY(nao_kinect_teleop SHARED
  src/tools.cpp
  src/utils/marker.cpp
  src/utils/ball-marker.cpp
  src/utils/line-marker.cpp
  src/utils/frame-marker.cpp
  src/utils/kinect-arm-points.cpp
  src/utils/nao-interface.cpp
  src/utils/joint-state-pub.cpp
  )

ADD_EXECUTABLE(nao_arms_teleop src/nao_arms_teleop.cpp)
TARGET_LINK_LIBRARIES(nao_arms_teleop
  nao_kinect_teleop
  ${catkin_LIBRARIES}
  ${OSCR_LIBRARIES}
)

add_executable(nao_teleop_arms_rviz src/teleop-arms-rviz.cpp)
TARGET_LINK_LIBRARIES(nao_teleop_arms_rviz
  nao_kinect_teleop
  ${catkin_LIBRARIES}
  ${OSCR_LIBRARIES}
)

add_executable(nao_teleop_arms src/teleop-arms.cpp)
TARGET_LINK_LIBRARIES(nao_teleop_arms
  nao_kinect_teleop
  ${catkin_LIBRARIES}
  ${OSCR_LIBRARIES}
)
