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
  src/utils/marker.cpp
  src/utils/ball-marker.cpp
  src/utils/line-marker.cpp
  src/utils/frame-marker.cpp
  src/utils/kinect-arm-points.cpp
  src/utils/nao-interface.cpp
  src/utils/joint-state-pub.cpp
  src/utils/kinect-arm-head-points.cpp
  )

SET(ALL_LIBRARIES
  nao_kinect_teleop
  ${catkin_LIBRARIES}
  ${OSCR_LIBRARIES}
)

# Deprecated example
ADD_EXECUTABLE(deprec_nao_arms_teleop
  src/deprecated/nao_arms_teleop.cpp
  src/deprecated/tools.cpp)
TARGET_LINK_LIBRARIES(deprec_nao_arms_teleop ${ALL_LIBRARIES})

# Executable to teleoperate the arms with RViz
ADD_EXECUTABLE(nao_teleop_arms_rviz src/teleop-arms-rviz.cpp)
TARGET_LINK_LIBRARIES(nao_teleop_arms_rviz ${ALL_LIBRARIES})

# Executable to teleoperate the arms with Naoqi (and the real robot)
ADD_EXECUTABLE(nao_teleop_arms src/teleop-arms.cpp)
TARGET_LINK_LIBRARIES(nao_teleop_arms ${ALL_LIBRARIES})

# Executable to teleoperate arms and head with Naoqi (real robot) 
ADD_EXECUTABLE(nao_teleop_arms_head src/teleop-arms-head.cpp)
TARGET_LINK_LIBRARIES(nao_teleop_arms_head ${ALL_LIBRARIES})

# Executable to teleoperate arms and head with Naoqi (real robot) 
# It uses BodyArray2 (with orientations)
ADD_EXECUTABLE(tmp src/teleop-arms-head-2.cpp)
TARGET_LINK_LIBRARIES(tmp ${ALL_LIBRARIES})
