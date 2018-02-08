
#include <nao_kinect_teleop/kinect-skeleton-fixed.hpp>


KinectSkeletonFixed::KinectSkeletonFixed()
  : msg_(new kinect_msgs::SkeletonFixedOrder)
{
  std::cout << msg_->position.size() << std::endl;
}


void KinectSkeletonFixed::readKinectPoints(
  const kinect_msgs::SkeletonFixedOrder::ConstPtr& msg
  )
{
  msg_ = msg;
}


kinect_msgs::SkeletonFixedOrder::ConstPtr KinectSkeletonFixed::getPoints()
{
  return msg_;
}

Eigen::Vector3d
KinectSkeletonFixed::getPointPositionById(const unsigned int& id)
{
  position_ << msg_->position[id].x, msg_->position[id].y, msg_->position[id].z;
  return position_;
}
