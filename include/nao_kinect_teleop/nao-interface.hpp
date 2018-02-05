#ifndef _NAO_KINECT_TELEOP_NAO_INTERFACE_HPP_
#define _NAO_KINECT_TELEOP_NAO_INTERFACE_HPP_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>


/**
 * Class that serves as interface between the controller (oscr) and naoqi (in
 * simulation and with the real robot)
 *
 */
class NaoInterface
{
public:

  /**
   * Constructor.
   *
   * @param[in] nh ROS node handle
   * @param[in] ndof_actuated number of actuated degrees of freedom
   * @param[in] jnames joint names as parsed by the robot model
   */
  NaoInterface(ros::NodeHandle& nh,
               const unsigned int& ndof_actuated_,
               const std::vector<std::string>& jnames);

  /**
   * Get the measured joint position values from the robot (or the simulation).
   *
   * @param[in] time Time in seconds that the system waits for a valid joint
   *                 position
   * @return measured joint positions
   */
  Eigen::VectorXd getMeasuredJointPositions(const double& time=1.0);

  /**
   * Set the command for the joint positions. This assumes that the robot is
   * position controlled.
   *
   * @param[in] q control values to be sent to the robot
   */
  void setJointPositionsCmd(const Eigen::VectorXd& q);

private:
  /// ROS node handle
  ros::NodeHandle* nh_;
  /// ROS publisher (to send the joint commands)
  ros::Publisher pub_;
  /// ROS subscriber (to read the measured joint values)
  ros::Subscriber sub_;
  /// Number of actuated degrees of freedom (must be 26 for nao)
  unsigned int ndof_actuated_;
  /// Message containing the sensed values
  sensor_msgs::JointState::ConstPtr jsensed_msg_;
  /// Message containing the joint commands
  naoqi_bridge_msgs::JointAnglesWithSpeed jcmd_msg_;

  /// Callback that copies the message to jsensed_msg_
  void callbackMeasuredJoint(const sensor_msgs::JointState::ConstPtr& msg);

};

/// Indices to convert from naoqi joint orders to oscr (urdf parser)
const unsigned int naoqi2oscr[26]= {
  0, 1, 8, 9, 10, 11, 12, 13, 2, 3, 4, 5, 6, 7,
  14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
};

#endif
