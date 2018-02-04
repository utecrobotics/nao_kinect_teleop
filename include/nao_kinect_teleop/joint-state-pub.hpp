#ifndef _NAO_KTELEOP_JOINT_STATE_PUB_HPP_
#define _NAO_KTELEOP_JOINT_STATE_PUB_HPP_


#include <Eigen/Dense>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

/**
 * Class to publish a joint configuration to the joint_states topic. This is
 * specific for the NAO robot.
 *
 */
class JointStatePub
{
public:
  /**
   * Constructor.
   * @param[in] nh ROS node handle
   * @param[in] ndof number of degrees of freedom for the message
   * @param[in] has_floating_base indication of floating base
   * @param[in] uses_reduced_model uses reduced model? (26 dofs)
   * @param[in] topic topic where to publish
   * @param[in] buff Buffer when publishing
   */
  JointStatePub(ros::NodeHandle& nh,
                const unsigned int& ndof,
                const bool& has_floating_base = false,
                const bool& uses_reduced_model = true,
                const std::string& topic = "joint_states",
                const unsigned int& buff = 1000);

  /**
   * Set names for the joints
   * @param[in] jnames Joint names
   */
  void setJointNames(const std::vector<std::string>& jnames);

  /**
   * Publish the joint configuration to the joint_states topic
   * @param[in] q Joint configuration (angles)
   */
  void publish(const Eigen::VectorXd& q);

private:
  /// Publisher
  ros::Publisher jstate_pub_;
  /// Joint States message
  sensor_msgs::JointState jstate_;
  /// Number of degrees of freedom
  unsigned int ndof_;
  /// Indication of floating base
  bool has_floating_base_;
  /// Indication of reduced model
  bool reduced_model_;
};

#endif
