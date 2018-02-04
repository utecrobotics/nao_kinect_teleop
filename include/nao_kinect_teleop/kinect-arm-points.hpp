#ifndef _NAO_KTELEOP_KINECT_ARM_POINTS_HPP_
#define _NAO_KTELEOP_KINECT_ARM_POINTS_HPP_


#include <kinect_msgs/BodyArray.h>


class KinectArmPoints
{
public:

  KinectArmPoints();

  void readKinectPoints(const kinect_msgs::BodyArray::ConstPtr& msg);

  kinect_msgs::BodyArray::ConstPtr getPoints();

private:
  kinect_msgs::BodyArray::ConstPtr msg_;

};


#endif
