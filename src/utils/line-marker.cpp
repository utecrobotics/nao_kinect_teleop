#include <nao_kinect_teleop/line-marker.hpp>


unsigned int LineMarker::id_=0;


LineMarker::LineMarker(ros::NodeHandle& nh,
                       double color[3],
                       const double& alpha,
                       const double& scale)
  : Marker(nh)
{
  marker_.header.frame_id = reference_frame_;
  marker_.ns = "line_markers";
  marker_.id = id_++;
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = 0.0;
  marker_.pose.position.y = 0.0;
  marker_.pose.position.z = 0.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = scale;   // m
  marker_.color.r = color[0];
  marker_.color.g = color[1];
  marker_.color.b = color[2];
  marker_.color.a = alpha;
  marker_.lifetime = ros::Duration();
}


void LineMarker::setPose(const Eigen::VectorXd& position)
{
  if (position.size()>=3)
  {
    geometry_msgs::Point p;
    p.x = position(0);
    p.y = position(1);
    p.z = position(2);
    marker_.points.push_back(p);
  }
  else
  {
    ROS_INFO("LineMarker: Not enough position elements.");
  }
}


void LineMarker::reset()
{
  marker_.points.clear();
}


void LineMarker::setColor(double color[3])
{
  marker_.color.r = color[0];
  marker_.color.g = color[1];
  marker_.color.b = color[2];
}


void LineMarker::publish()
{
  marker_pub_.publish(marker_);
}

