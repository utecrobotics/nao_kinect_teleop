#ifndef _NAO_KTELEOP_KINECT_ARM_HEAD_POINTS_HPP_
#define _NAO_KTELEOP_KINECT_ARM_HEAD_POINTS_HPP_


#include <Eigen/Dense>
#include <kinect_msgs/BodyArray2.h>


class KinectArmHeadPoints
{
public:

  KinectArmHeadPoints();

  void readKinectPoints(const kinect_msgs::BodyArray2::ConstPtr& msg);

  kinect_msgs::BodyArray2::ConstPtr getPoints();

  Eigen::Vector3d getPointPositionById(const unsigned int& id);

private:
  kinect_msgs::BodyArray2::ConstPtr msg_;
  Eigen::Vector3d position_;

};


#endif
