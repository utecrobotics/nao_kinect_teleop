#include <nao_kinect_teleop/joint-state-pub.hpp>


JointStatePub::JointStatePub(ros::NodeHandle& nh,
                             const unsigned int& ndof,
                             const bool& has_floating_base,
                             const bool& uses_reduced_model,
                             const std::string& topic,
                             const unsigned int& buff)
  : ndof_(ndof)
  , has_floating_base_(has_floating_base)
  , reduced_model_(uses_reduced_model)
{
  jstate_pub_ =
    nh.advertise<sensor_msgs::JointState>(topic, buff);

  unsigned int nfingers = reduced_model_? 16 : 0;
  jstate_.name.resize(nfingers+ndof_);
  jstate_.position.resize(nfingers+ndof_);
  jstate_.header.stamp = ros::Time::now();
  if (reduced_model_)
  {
    // Set the last 16 degrees of freedom (fingers) with zeros
    for (unsigned int i=0; i<16; ++i)
      jstate_.position[i+ndof_] = 0.0;
  }
}


void JointStatePub::setJointNames(const std::vector<std::string>& jnames)
{
  if (has_floating_base_)
  {
    if (true)
    {
      // When orientation is tiven by a quaternion
      std::string floating_base_names[7] =
        {"PX", "PY", "PZ", "EX", "EY", "EZ", "W"};
      for (unsigned int i=0; i<7; ++i)
        jstate_.name[i] = floating_base_names[i];
      for (unsigned int i=7; i<ndof_; ++i)
        jstate_.name[i] = jnames[i-7];
    }
    else
    {
      // When orientation is given by RPY angles
      std::string floating_base_names[6] =
        {"PX", "PY", "PZ", "RX", "RY", "RZ"};
      for (unsigned int i=0; i<6; ++i)
        jstate_.name[i] = floating_base_names[i];
      for (unsigned int i=6; i<ndof_; ++i)
        jstate_.name[i] = jnames[i-6];
    }
  }
  else
  {
    for (unsigned int i=0; i<ndof_; ++i)
      jstate_.name[i] = jnames[i];
  }
  if (reduced_model_)
  {
    // Add joint names for the fingers explicitly when the reduced model is
    // used
    std::string finger_names[16] = {
      "LFinger11","LFinger12","LFinger13","LFinger21",
      "LFinger22","LFinger23","LThumb1","LThumb2",
      "RFinger11","RFinger12","RFinger13","RFinger21",
      "RFinger22","RFinger23","RThumb1","RThumb2"};
    for (unsigned int i=1; i<16; ++i)
      jstate_.name[ndof_+i] = finger_names[i];
  }
}


void JointStatePub::publish(const Eigen::VectorXd& q)
{
  for (unsigned int i=0; i<ndof_; ++i)
    jstate_.position[i] = q[i];

  jstate_.header.stamp = ros::Time::now();
  jstate_pub_.publish(jstate_);
}
