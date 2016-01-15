#ifndef NAO_TOOLS_H
#define NAO_TOOLS_H


#include <sensor_msgs/JointState.h>
#include <nao_kinect_teleop/robotSpecifics.h>
#include <Eigen/Dense>


/// Class to read the joint sensor messages from NAO (virtual or real)
class JointSensors
{
public:
  /**
   * Constructor
   */
  JointSensors();
  /***
   * Callback function that copies the message to an internal member
   */
  void readJointSensors(const sensor_msgs::JointState::ConstPtr& msg);
  /**
   * Get the joint state full message read from the topic
   */
  sensor_msgs::JointState::ConstPtr sensedValue();
  /**
   * Get NAO sensed joints in RBDL reduced format (without fingers)
   * @param[out] jsensed sensed joints
   * @param[in] ndof expected number of dofs
   */
  void getSensedJointsRBDL(Eigen::VectorXd& jsensed,
                           const unsigned int& ndof);

private:
  /// Sensed joints
  sensor_msgs::JointState::ConstPtr msg_;
};

/**
 * Convert full joints (42) to reduced joints (26)
 * @param[out] jreduced reduced joints (RBDL model order)
 * @param[in] jfull full joints (RBDL model order)
 */
void reducedJointModel(std::vector<float>& jreduced,
                       const Eigen::VectorXd& jfull);



#endif
