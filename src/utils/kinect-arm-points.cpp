
#include <nao_kinect_teleop/kinect-arm-points.hpp>


KinectArmPoints::KinectArmPoints()
  : msg_(new kinect_msgs::SkeletonFixedOrder)
{
  std::cout << msg_->position.size() << std::endl;
}


void KinectArmPoints::readKinectPoints(
  const kinect_msgs::SkeletonFixedOrder::ConstPtr& msg
  )
{
  msg_ = msg;
}


kinect_msgs::SkeletonFixedOrder::ConstPtr KinectArmPoints::getPoints()
{
  return msg_;
}

Eigen::Vector3d KinectArmPoints::getPointPositionById(const unsigned int& id)
{
  position_ << msg_->position[id].x, msg_->position[id].y, msg_->position[id].z;
  return position_;
}
