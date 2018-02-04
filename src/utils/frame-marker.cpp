#include <nao_kinect_teleop/frame-marker.hpp>
#include <oscr/tools/math-utils.hpp>


unsigned int FrameMarker::id_=0;


FrameMarker::FrameMarker(ros::NodeHandle& nh,
                         const double& color_saturation,
                         const double& alpha,
                         const double& scale)
  : Marker(nh)
{
  markerx_.header.frame_id = reference_frame_;
  markery_.header.frame_id = reference_frame_;
  markerz_.header.frame_id = reference_frame_;
  markerx_.ns = "frame_markers";
  markery_.ns = "frame_markers";
  markerz_.ns = "frame_markers";
  markerx_.id = id_++;
  markery_.id = id_++;
  markerz_.id = id_++;
  markerx_.type = visualization_msgs::Marker::ARROW;
  markery_.type = visualization_msgs::Marker::ARROW;
  markerz_.type = visualization_msgs::Marker::ARROW;
  markerx_.action = visualization_msgs::Marker::ADD;
  markery_.action = visualization_msgs::Marker::ADD;
  markerz_.action = visualization_msgs::Marker::ADD;
  markerx_.pose.position.x = 0.0;
  markerx_.pose.position.y = 0.0;
  markerx_.pose.position.z = 0.0;
  markerx_.pose.orientation.w = 1.0;
  markerx_.pose.orientation.x = 0.0;
  markerx_.pose.orientation.y = 0.0;
  markerx_.pose.orientation.z = 0.0;
  markery_.pose.position.x = 0.0;
  markery_.pose.position.y = 0.0;
  markery_.pose.position.z = 0.0;
  markery_.pose.orientation.w = cos(M_PI/4.0);
  markery_.pose.orientation.x = 0.0;
  markery_.pose.orientation.y = 0.0;
  markery_.pose.orientation.z = sin(M_PI/4.0);
  markerz_.pose.position.x = 0.0;
  markerz_.pose.position.y = 0.0;
  markerz_.pose.position.z = 0.0;
  markerz_.pose.orientation.w = cos(-M_PI/4.0);
  markerz_.pose.orientation.x = 0.0;
  markerz_.pose.orientation.y = sin(-M_PI/4.0);
  markerz_.pose.orientation.z = 0.0;
  markerx_.scale.x = scale; // m
  markerx_.scale.y = 0.01;  // m
  markerx_.scale.z = 0.01;  // m
  markery_.scale.x = scale; // m
  markery_.scale.y = 0.01;  // m
  markery_.scale.z = 0.01;  // m
  markerz_.scale.x = scale; // m
  markerz_.scale.y = 0.01;  // m
  markerz_.scale.z = 0.01;  // m
  markerx_.color.r = color_saturation;
  markerx_.color.g = 0.0;
  markerx_.color.b = 0.0;
  markerx_.color.a = alpha;
  markery_.color.r = 0.0;
  markery_.color.g = color_saturation;
  markery_.color.b = 0.0;
  markery_.color.a = alpha;
  markerz_.color.r = 0.0;
  markerz_.color.g = 0.0;
  markerz_.color.b = color_saturation;
  markerz_.color.a = alpha;
  markerx_.lifetime = ros::Duration();
  markery_.lifetime = ros::Duration();
  markerz_.lifetime = ros::Duration();
}


void FrameMarker::setPose(const Eigen::VectorXd& pose)
{
  markerx_.pose.position.x = pose(0);
  markerx_.pose.position.y = pose(1);
  markerx_.pose.position.z = pose(2);

  markery_.pose.position.x = pose(0);
  markery_.pose.position.y = pose(1);
  markery_.pose.position.z = pose(2);

  markerz_.pose.position.x = pose(0);
  markerz_.pose.position.y = pose(1);
  markerz_.pose.position.z = pose(2);

  if (pose.size()==7)
  {
    // When orientation is in roll, pitch, yaw format
    // Eigen::Matrix3d R;
    // R = Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitZ())
    //     * Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitX());

    // X is aligned and has no rotation
    markerx_.pose.orientation.w = pose(3);
    markerx_.pose.orientation.x = pose(4);
    markerx_.pose.orientation.y = pose(5);
    markerx_.pose.orientation.z = pose(6);

    // Y is rotated 90 wrt current Z
    Eigen::Vector4d q, q1;
    q1 << cos(M_PI/4.0), 0.0, 0.0, sin(M_PI/4.0);
    q = oscr::quaternionMult(pose.tail(4), q1);
    markery_.pose.orientation.w = q(0);
    markery_.pose.orientation.x = q(1);
    markery_.pose.orientation.y = q(2);
    markery_.pose.orientation.z = q(3);

    // Z is rotated -90 wrt current Y
    q1 << cos(-M_PI/4.0), 0.0, sin(-M_PI/4.0), 0.0;
    q = oscr::quaternionMult(pose.tail(4), q1);
    markerz_.pose.orientation.w = q(0);
    markerz_.pose.orientation.x = q(1);
    markerz_.pose.orientation.y = q(2);
    markerz_.pose.orientation.z = q(3);
  }
  publish();
}


void FrameMarker::publish()
{
  marker_pub_.publish(markerx_);
  marker_pub_.publish(markery_);
  marker_pub_.publish(markerz_);
}
