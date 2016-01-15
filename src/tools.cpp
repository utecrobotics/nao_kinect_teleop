#include <nao_kinect_teleop/tools.hpp>


JointSensors::JointSensors()
  : msg_(new sensor_msgs::JointState)
{
}

void JointSensors::readJointSensors(const sensor_msgs::JointState::ConstPtr& msg)
{
  msg_ = msg;
}

sensor_msgs::JointState::ConstPtr JointSensors::sensedValue()
{
  return msg_;
}

void JointSensors::getSensedJointsRBDL(Eigen::VectorXd& jsensed,
                                       const unsigned int& ndof)
{
  for (unsigned int i=0; i<ndof; ++i)
  {
    // The index 'jidx' maps joint names from the sensed values to the internal
    // sensed values used with RBDL model (42 dofs, joint order different)
    jsensed[jidx[i]] = msg_->position[i];
  }
}



void reducedJointModel(std::vector<float>& jreduced,
                       const Eigen::VectorXd& jfull)
{
  for (unsigned int i=0; i<jfull.size(); ++i)
  {
    if (ridx[i] != 100)
      jreduced[ridx[i]] = jfull[i];
  }
}
