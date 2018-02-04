#ifndef _NAO_KINECT_TELEOP_FRAME_MARKER_HPP_
#define _NAO_KINECT_TELEOP_FRAME_MARKER_HPP_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nao_kinect_teleop/marker.hpp>

/**
 * Class to show frames as markers
 *
 */
class FrameMarker
  : public Marker
{
public:

  /**
   * Constructor.
   * @param[in] nh ROS node handle
   * @param[in] color_saturation indication of color saturation
   * @param[in] alpha indication of color transparency (alpha)
   * @param[in] scale ball size (scaled from 1m) in m
   */
  FrameMarker(ros::NodeHandle& nh,
              const double& color_saturation=1.0,
              const double& alpha=1.0,
              const double& scale=0.1);

  // Inherited members
  void setPose(const Eigen::VectorXd& pose);
  void publish();

private:
  /// Marker for x axis
  visualization_msgs::Marker markerx_;
  /// Marker for y axis
  visualization_msgs::Marker markery_;
  /// Marker for z axis
  visualization_msgs::Marker markerz_;
  /// Frame marker unique identifier
 static unsigned int id_;
};

#endif
