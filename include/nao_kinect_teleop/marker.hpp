#ifndef _NAO_KINECT_TELEOP_MARKER_HPP_
#define _NAO_KINECT_TELEOP_MARKER_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>

/**
 * Base class to visualize markers
 *
 */
class Marker
{
public:

  /**
   * Constructor.
   * @param[in] nh ROS node handle
   */
  Marker(ros::NodeHandle& nh);

  /**
   * Set the marker pose (position + orientation, if present)
   * @param[in] pose vector of position and orientation (if present)
   */
  virtual void setPose(const Eigen::VectorXd& pose) = 0;

  /**
   * Publish the marker to the visualization_marker topic
   */
  virtual void publish() = 0;

protected:
  /// Publisher
  ros::Publisher marker_pub_;
  /// Reference frame
  std::string reference_frame_;
};


/**
 * Helper for marker colors
 *
 */
static double RED[3]       = {1.0, 0.0, 0.0};
static double GREEN[3]     = {0.0, 1.0, 0.0};
static double BLUE[3]      = {0.0, 0.0, 1.0};
static double YELLOW[3]    = {1.0, 1.0, 0.0};
static double PINK[3]      = {1.0, 0.0, 1.0};
static double CYAN[3]      = {0.0, 1.0, 1.0};
static double BLACK[3]     = {0.0, 0.0, 0.0};
static double DARKGRAY[3]  = {0.2, 0.2, 0.2};
static double LIGHTGRAY[3] = {0.5, 0.5, 0.5};
static double WHITE[3]     = {1.0, 1.0, 1.0};



#endif
