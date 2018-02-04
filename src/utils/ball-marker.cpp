#include <nao_kinect_teleop/ball-marker.hpp>


unsigned int BallMarker::id_=0;


BallMarker::BallMarker(ros::NodeHandle& nh,
                       double color[3],
                       const double& alpha,
                       const double& scale)
  : Marker(nh)
{
  marker_.header.frame_id = reference_frame_;
  marker_.ns = "ball_markers";
  marker_.id = id_++;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = 0.0;
  marker_.pose.position.y = 0.0;
  marker_.pose.position.z = 0.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = scale;   // m
  marker_.scale.y = scale;   // m
  marker_.scale.z = scale;   // m
  marker_.color.r = color[0];
  marker_.color.g = color[1];
  marker_.color.b = color[2];
  marker_.color.a = alpha;
  marker_.lifetime = ros::Duration();
}


void BallMarker::setPose(const Eigen::VectorXd& position)
{
  if (position.size()>=3)
  {
    marker_.pose.position.x = position(0);
    marker_.pose.position.y = position(1);
    marker_.pose.position.z = position(2);
    // Publish the passed pose
    publish();
  }
  else
  {
    ROS_INFO("BallMarker: Not enough position elements.");
  }
}


void BallMarker::publish()
{
  marker_pub_.publish(marker_);
}

