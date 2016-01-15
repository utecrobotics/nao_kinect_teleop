/*                                                                               
 * Copyright 2016
 * J.Avalos, S.Cortez, O.Ramos. 
 * Universidad de Ingenieria y Tecnologia - UTEC
 *
 * This file is part of nao_kinect_teleop.
 * nao_kinect_teleop is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * nao_kinect_teleop is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with nao_kinect_teleop. If not, see <http://www.gnu.org/licenses/>.
 */

#include <nao_kinect_teleop/markers.hpp>
#include <Eigen/Geometry>


Marker::Marker(ros::NodeHandle& nh)
{
  nh.param<std::string>("reference_frame", reference_frame_, "/map");
  marker_pub_ = 
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}


unsigned int BallMarker::id_=0;

BallMarker::BallMarker(ros::NodeHandle& nh, 
                       double color[3],
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
  marker_.color.a = 1.0;
  marker_.lifetime = ros::Duration();
}


void BallMarker::setPose(const Eigen::VectorXd& position)
{
  marker_.pose.position.x = position(0);
  marker_.pose.position.y = position(1);
  marker_.pose.position.z = position(2);
}


void BallMarker::publish()
{
  marker_pub_.publish(marker_);
}


unsigned int FrameMarker::id_=0;

FrameMarker::FrameMarker(ros::NodeHandle& nh,
                         const double& color_saturation,
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
  markerx_.pose.orientation.x = 1.0;
  markerx_.pose.orientation.y = 0.0;
  markerx_.pose.orientation.z = 0.0;
  markerx_.pose.orientation.w = 1.0;
  markery_.pose.position.x = 0.0;
  markery_.pose.position.y = 0.0;
  markery_.pose.position.z = 0.0;
  markery_.pose.orientation.x = 0.0;
  markery_.pose.orientation.y = 1.0;
  markery_.pose.orientation.z = 0.0;
  markery_.pose.orientation.w = 1.0;
  markerz_.pose.position.x = 0.0;
  markerz_.pose.position.y = 0.0;
  markerz_.pose.position.z = 0.0;
  markerz_.pose.orientation.x = 0.0;
  markerz_.pose.orientation.y = 0.0;
  markerz_.pose.orientation.z = 1.0;
  markerz_.pose.orientation.w = 1.0;
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
  markerx_.color.a = 1.0;
  markery_.color.r = 0.0;
  markery_.color.g = color_saturation;
  markery_.color.b = 0.0;
  markery_.color.a = 1.0;
  markerz_.color.r = 0.0;
  markerz_.color.g = 0.0;
  markerz_.color.b = color_saturation;
  markerz_.color.a = 1.0;
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

  if (pose.size()==6)
  {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q;
    q = R;
    markerx_.pose.orientation.x = q.x();
    markerx_.pose.orientation.y = q.y();
    markerx_.pose.orientation.z = q.z();
    markerx_.pose.orientation.w = q.w();

    q = R*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
    markery_.pose.orientation.x = q.x();
    markery_.pose.orientation.y = q.y();
    markery_.pose.orientation.z = q.z();
    markery_.pose.orientation.w = q.w();

    q = R*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
    markerz_.pose.orientation.x = q.x();
    markerz_.pose.orientation.y = q.y();
    markerz_.pose.orientation.z = q.z();
    markerz_.pose.orientation.w = q.w();
  }
}


void FrameMarker::publish()
{
  marker_pub_.publish(markerx_);
  marker_pub_.publish(markery_);
  marker_pub_.publish(markerz_);
}
