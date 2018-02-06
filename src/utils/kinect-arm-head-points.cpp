
#include <nao_kinect_teleop/kinect-arm-head-points.hpp>


KinectArmHeadPoints::KinectArmHeadPoints()
  : msg_(new kinect_msgs::BodyArray2)
{
  std::cout << msg_->body.size() << std::endl;
}


void KinectArmHeadPoints::readKinectPoints(
  const kinect_msgs::BodyArray2::ConstPtr& msg)
{
  msg_ = msg;
}


kinect_msgs::BodyArray2::ConstPtr KinectArmHeadPoints::getPoints()
{
  return msg_;
}

Eigen::Vector3d KinectArmHeadPoints::getPointPositionById(const unsigned int& id)
{
  position_ << msg_->body[id].x, msg_->body[id].y, msg_->body[id].z;
  return position_;
}
