nao_kinect_teleop - Package to teleoperate NAO using the Kinect

Introduction
============

nao_kinect_teleop is a ROS package that allows to teleoperate the NAO robot
using the motion shown by a person. The motion is acquired using a Microsoft
Kinect sensor (in a windows host) and is sent to ROS using rosserial.

### Dependencies

The nao_kinect_teleop package dependds on the following libraries/packages
which have to be available in your machine.

  - Libraries:
    - rbdl (with urdf support)
    - qpOASES
    - Eigen (>=3.2)
  - ROS Packages:
    - osik_control (https://github.com/utecrobotics/osik-control)
    - kinect_msgs (https://github.com/utecrobotics/kinect_msgs)
    - rosserial (https://github.com/ros-drivers/rosserial)
    - nao ROS Support (For details, see http://wiki.ros.org/nao)
  - Other Software:
    - NAOqi



