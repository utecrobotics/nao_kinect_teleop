#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


class Marker
{
public:

  Marker(ros::NodeHandle& nh);
  virtual void setPose(const Eigen::VectorXd& position) = 0;
  virtual void publish() = 0;

protected:

  ros::Publisher marker_pub_;
  std::string reference_frame_;
};


class BallMarker
  : public Marker
{
public:
  BallMarker(ros::NodeHandle& nh, 
             double color[3],
             const double& scale=0.05);

  void setPose(const Eigen::VectorXd& position);

  void publish();

private:

  visualization_msgs::Marker marker_;
  static unsigned int id_;
};


class FrameMarker
  : public Marker
{
public:
  FrameMarker(ros::NodeHandle& nh,
              const double& color_saturation=1.0,
              const double& scale=0.1);

  void setPose(const Eigen::VectorXd& pose);
  
  void publish();

private:

  visualization_msgs::Marker markerx_;
  visualization_msgs::Marker markery_;
  visualization_msgs::Marker markerz_;
  static unsigned int id_;
};


