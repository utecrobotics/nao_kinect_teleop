
#include <nao_kinect_teleop/kinect-arm-points.hpp>


KinectArmPoints::KinectArmPoints()
  : msg_(new kinect_msgs::BodyArray)
{
  std::cout << msg_->body.size() << std::endl;
}


void KinectArmPoints::readKinectPoints(const kinect_msgs::BodyArray::ConstPtr& msg)
{
  msg_ = msg;
}


kinect_msgs::BodyArray::ConstPtr KinectArmPoints::getPoints()
{
  return msg_;
}

Eigen::Vector3d KinectArmPoints::getPointPositionById(const unsigned int& id)
{
  position_ << msg_->body[id].x, msg_->body[id].y, msg_->body[id].z;
  return position_;
}
