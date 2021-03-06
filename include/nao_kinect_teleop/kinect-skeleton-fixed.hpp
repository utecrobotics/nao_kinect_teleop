#ifndef _NAO_KTELEOP_KINECT_SKELETON_FIXED_HPP_
#define _NAO_KTELEOP_KINECT_SKELETON_FIXED_HPP_


#include <Eigen/Dense>
#include <kinect_msgs/SkeletonFixedOrder.h>


class KinectSkeletonFixed
{
public:

  KinectSkeletonFixed();

  void readKinectPoints(const kinect_msgs::SkeletonFixedOrder::ConstPtr& msg);

  kinect_msgs::SkeletonFixedOrder::ConstPtr getPoints();

  Eigen::Vector3d getPointPositionById(const unsigned int& id);

private:
  kinect_msgs::SkeletonFixedOrder::ConstPtr msg_;
  Eigen::Vector3d position_;

};


#endif
