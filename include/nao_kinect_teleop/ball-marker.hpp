#ifndef _NAO_KINECT_TELEOP_BALL_MARKER_HPP_
#define _NAO_KINECT_TELEOP_BALL_MARKER_HPP_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nao_kinect_teleop/marker.hpp>


/**
 * Class to visualize ball markers
 *
 */
class BallMarker
  : public Marker
{
public:

  /**
   * Constructor.
   *
   * @param[in] nh ROS node handle
   * @param[in] color indication of color in RGB
   * @param[in] alpha indication of color transparency (alpha)
   * @param[in] scale ball size (scaled from 1m) in m
   */
  BallMarker(ros::NodeHandle& nh,
             double color[3],
             const double& alpha=1.0,
             const double& scale=0.05);

  // Inherited members
  void setPose(const Eigen::VectorXd& position);
  void publish();

private:
  /// Ball marker message
  visualization_msgs::Marker marker_;
  /// Ball marker unique identifier
  static unsigned int id_;
};

#endif
