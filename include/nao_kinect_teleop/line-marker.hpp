#ifndef _NAO_KINECT_TELEOP_LINE_MARKER_HPP_
#define _NAO_KINECT_TELEOP_LINE_MARKER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nao_kinect_teleop/marker.hpp>


/**
 * Class to visualize line markers
 *
 */
class LineMarker
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
  LineMarker(ros::NodeHandle& nh,
             double color[3],
             const double& alpha=1.0,
             const double& scale=0.01);

  // Inherited members
  void reset();
  void setPose(const Eigen::VectorXd& position);
  void publish();
  void setColor(double color[3]);

private:
  /// Ball marker message
  visualization_msgs::Marker marker_;
  /// Line marker unique identifier
  static unsigned int id_;
};

#endif
