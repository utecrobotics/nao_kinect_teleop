CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(nao_kinect_teleop)

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Operational Space Inverse Kinematics Control and RBDL packages
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(OSIK_CONTROL REQUIRED osik-control)
PKG_CHECK_MODULES(RBDL REQUIRED rbdl)
IF (RBDL_FOUND)
  SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
ENDIF (RBDL_FOUND)

# Find QPOASES
FIND_PACKAGE(qpOASES)

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
  ${RBDL_INCLUDE_DIRS}
  ${OSIK_CONTROL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${OSIK_CONTROL_LIBRARY_DIRS}
)

ADD_LIBRARY(nao_kinect_teleop SHARED
  src/tools.cpp
  src/markers.cpp
  )


ADD_EXECUTABLE(nao_arms_teleop src/nao_arms_teleop.cpp)
TARGET_LINK_LIBRARIES(nao_arms_teleop
  nao_kinect_teleop
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARIES}
  ${RBDL_URDFReader_LIBRARIES}
  ${OSIK_CONTROL_LIBRARIES}
  ${qpOASES_LIBRARIES}
)
