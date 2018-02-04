#include <visualization_msgs/Marker.h>
#include <nao_kinect_teleop/marker.hpp>


Marker::Marker(ros::NodeHandle& nh)
{
  // Get the reference frame form the parameter server
  nh.param<std::string>("reference_frame", reference_frame_, "/map");
  // Publisher to the visualization_marker topic
  marker_pub_ =
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

