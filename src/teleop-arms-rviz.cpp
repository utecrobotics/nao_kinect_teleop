/***
 * Copyright 2018
 * J.Avalos, O.Ramos
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
 *
 * -------------------------------------------------------------------------
 *
 * Teleoperation of NAO arms with a Kinect v2 using only the kinematic model
 * in RViz (there is no interaction with naoqi).
 *
 * -------------------------------------------------------------------------
 */


#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

#include <oscr/oscr.hpp>

#include <nao_kinect_teleop/joint-state-pub.hpp>
#include <nao_kinect_teleop/markers.hpp>
#include <nao_kinect_teleop/kinect-skeleton-fixed.hpp>


int main(int argc, char **argv)
{
  // Load the urdf model
  // --------------------------------------------------------------------
  // The robot is assumed to be fixed on the ground
  bool has_floating_base = false;
  // Path to the robot URDF (reduced with only 26 dofs)
  std::string model_pkg = ros::package::getPath("nao_kinect_teleop");
  std::string model_name = model_pkg + "/urdf/naoV40red.urdf";
  // Load the robot model (use RobotModelPin or RobotModelRbdl)
  oscr::RobotModel* robot = new oscr::RobotModelRbdl(model_name,
                                                     has_floating_base);
  // --------------------------------------------------------------------

  // Information about joints and their limits
  std::vector<std::string> jnames = robot->jointNames();
  std::map<std::string, unsigned int> mlink = robot->mapLinkNamesIDs();
  // Joint limits (only needed for WQP and HQP)
  Eigen::VectorXd qmin, qmax, dqmax;
  qmin = robot->jointMinAngularLimits();
  qmax = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();

  // Initialize ROS
  ros::init(argc, argv, "naoTeleopArmsRviz");
  ros::NodeHandle nh;

  // Initialize publishers and subscribers
  // --------------------------------------------------------------------
  // * Publisher of  Joint States (for rviz)
  JointStatePub jstate_pub(nh, robot->ndof(), has_floating_base);
  jstate_pub.setJointNames(jnames);
  // * Subscriber to kinect messages
  KinectSkeletonFixed kpoints;
  ros::Subscriber sub = nh.subscribe("kinect_skeleton", 1000,
                                     &KinectSkeletonFixed::readKinectPoints,
                                     &kpoints);

  // Assign the initial joint configuration
  Eigen::VectorXd q(robot->ndof());
  q << 0.0, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    1.15,  0.10, -1.4, -0.79, 0.0, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    1.15, -0.10,  1.4,  0.79, 0.0, 0.0;
  robot->updateJointConfig(q);
  // Publish the initial configuration
  jstate_pub.publish(q);

  // Inverse Kinematics tasks
  // --------------------------------------------------------------------
  oscr::KineTask* taskrh;
  oscr::KineTask* tasklh;
  oscr::KineTask* taskre;
  oscr::KineTask* taskle;
  taskrh = new oscr::KineTaskPose(robot, mlink["r_gripper"], "position");
  taskrh->setGain(10.0);
  tasklh = new oscr::KineTaskPose(robot, mlink["l_gripper"], "position");
  tasklh->setGain(10.0);
  taskre = new oscr::KineTaskPose(robot, mlink["RElbow"], "position");
  taskre->setGain(10.0);
  taskle = new oscr::KineTaskPose(robot, mlink["LElbow"], "position");
  taskle->setGain(10.0);

  // Operational-Space Inverse Kinematics (OSIK) solver
  // --------------------------------------------------------------------
  // Sampling time
  unsigned int f = 100;
  double dt = static_cast<double>(1.0/f);
  // Solver (WQP, HQP or NS)
  oscr::OSIKSolverWQP solver(robot, q, dt);
  solver.setJointLimits(qmin, qmax, dqmax);
  // Add tasks to the solver
  solver.pushTask(taskrh);
  solver.pushTask(tasklh);
  solver.pushTask(taskre);
  solver.pushTask(taskle);

  // Ball markers for the human skeleton
  std::vector<BallMarker*> sk_bmarkers;
  sk_bmarkers.resize(6);
  for (unsigned int i=0; i<sk_bmarkers.size(); ++i)
    sk_bmarkers.at(i) = new BallMarker(nh, RED);
  // Line markers for the human skeleton
  LineMarker sk_lmarkers(nh, GREEN);
  // Ball markers for the retargeted (nao) skeleton
  std::vector<BallMarker*> nao_markers;
  nao_markers.resize(6);
  for (unsigned int i=0; i<nao_markers.size(); ++i)
    nao_markers.at(i) = new BallMarker(nh, GREEN);

  // TaskMarkers markers(nh);
  // markers.add(tasklh);
  // markers.add(taskrh);

  // Nao lengths
  double Lnao_upperarm = 0.108;  // From shoulder to elbow
  double Lnao_forearm  = 0.111;  // From elbow to hand

  // Vectors for positions/poses
  Eigen::VectorXd p_rshoulder(3), p_relbow(3), p_rwrist(3);
  Eigen::VectorXd p_lshoulder(3), p_lelbow(3), p_lwrist(3);
  Eigen::VectorXd sk_rshoulder(3), sk_relbow(3), sk_rwrist(3);
  Eigen::VectorXd sk_lshoulder(3), sk_lelbow(3), sk_lwrist(3);
  // Vector for the desired joint configuration
  Eigen::VectorXd qdes;

  ros::Rate rate(f); // Hz
  while(ros::ok())
  {
    unsigned int n_kinect_points = kpoints.getPoints()->position.size();

    // Only work when there are skeleton points from the Kinect
    if (n_kinect_points > 0)
    {
      ROS_INFO_ONCE("Kinect point values found!");

      // Human skeleton points
      // ------------------------------------------------------------------
      // Get the human skeleton positions
      sk_rshoulder = kpoints.getPointPositionById(0);
      sk_relbow    = kpoints.getPointPositionById(1);
      sk_rwrist    = kpoints.getPointPositionById(2);
      sk_lshoulder = kpoints.getPointPositionById(3);
      sk_lelbow    = kpoints.getPointPositionById(4);
      sk_lwrist    = kpoints.getPointPositionById(5);
      // Show the ball markers corresponding to the human skeleton
      sk_bmarkers.at(0)->setPose(sk_rshoulder);
      sk_bmarkers.at(1)->setPose(sk_relbow);
      sk_bmarkers.at(2)->setPose(sk_rwrist);
      sk_bmarkers.at(3)->setPose(sk_lshoulder);
      sk_bmarkers.at(4)->setPose(sk_lelbow);
      sk_bmarkers.at(5)->setPose(sk_lwrist);
      // Reset the line markers
      sk_lmarkers.reset();
      // Show the skeleton lines
      sk_lmarkers.setPose(sk_rshoulder); sk_lmarkers.setPose(sk_relbow);
      sk_lmarkers.setPose(sk_relbow);    sk_lmarkers.setPose(sk_rwrist);
      sk_lmarkers.setPose(sk_lshoulder); sk_lmarkers.setPose(sk_lelbow);
      sk_lmarkers.setPose(sk_lelbow);    sk_lmarkers.setPose(sk_lwrist);
      sk_lmarkers.setPose(sk_rshoulder); sk_lmarkers.setPose(sk_lshoulder);
      sk_lmarkers.publish();

      // Right arm
      // ------------------------------------------------------------------
      // Positions with respect to the right shoulder
      p_rshoulder = sk_rshoulder - sk_rshoulder;
      p_relbow = sk_relbow - sk_rshoulder;
      p_rwrist = sk_rwrist - sk_rshoulder;
      // Length ratio: (Nao limbs)/(human skeleton limbs)
      double k_rupperarm = Lnao_upperarm / p_relbow.norm();
      double k_rforearm  = Lnao_forearm / (p_rwrist-p_relbow).norm();
      // Nao right shoulder (constant) position in base frame
      p_rshoulder << 0.0, -0.098, 0.1;
      // Wrist wrt elbow retargeting the length to NAO
      p_rwrist = k_rforearm*(p_rwrist-p_relbow);
      // Elbow retargeting the length to NAO, in base frame
      p_relbow = p_rshoulder + k_rupperarm*p_relbow;
      // Wrist in base frame
      p_rwrist = p_rwrist + p_relbow;

      // Left arm
      // ------------------------------------------------------------------
      // Positions with respect to the left shoulder
      p_lshoulder = sk_lshoulder - sk_lshoulder;
      p_lelbow = sk_lelbow - sk_lshoulder;
      p_lwrist = sk_lwrist - sk_lshoulder;
      // Length ratio: (Nao limbs)/(human skeleton limbs)
      double k_lupperarm = Lnao_upperarm / p_lelbow.norm();
      double k_lforearm  = Lnao_forearm / (p_lwrist-p_lelbow).norm();
      // Nao left shoulder (constant) position in base frame
      p_lshoulder << 0.0, 0.098, 0.1;
      // Wrist wrt elbow retargeting the length to NAO
      p_lwrist = k_lforearm*(p_lwrist-p_lelbow);
      // Elbow retargeting the length to NAO, in base frame
      p_lelbow = p_lshoulder + k_lupperarm*p_lelbow;
      // Wrist in base frame
      p_lwrist = p_lwrist + p_lelbow;

      // Show markers for NAO
      nao_markers.at(0)->setPose(p_relbow);
      nao_markers.at(1)->setPose(p_rwrist);
      nao_markers.at(2)->setPose(p_rshoulder);
      nao_markers.at(3)->setPose(p_lelbow);
      nao_markers.at(4)->setPose(p_lwrist);
      nao_markers.at(5)->setPose(p_lshoulder);

      // Set the desired positions for the tasks
      taskre->setDesiredValue(p_relbow);
      taskrh->setDesiredValue(p_rwrist);
      taskle->setDesiredValue(p_lelbow);
      tasklh->setDesiredValue(p_lwrist);

      // Get the inverse kinematics solution
      solver.getPositionControl(q, qdes);
      robot->updateJointConfig(q);
      jstate_pub.publish(robot->getJointConfig());
      // sk_bmarkers.update();
      q = qdes;
    }
    else
    {

      ROS_WARN_THROTTLE(1, "Not enough (%d) skeleton points from kinect found",
                        n_kinect_points);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


