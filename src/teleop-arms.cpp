/*
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
 * Teleoperation of NAO arms with a Kinect v2 using Naoqi (either the "real
 * robot" or a simulated "naoqi robot"
 *
 * -------------------------------------------------------------------------
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <oscr/oscr.hpp>

#include <nao_kinect_teleop/kinect-arm-points.hpp>
#include <nao_kinect_teleop/nao-interface.hpp>
#include <nao_kinect_teleop/markers.hpp>


int main(int argc, char **argv)
{
  // Load the reduced urdf model (only 26 dofs, without fingers)
  // --------------------------------------------------------------------
  // The robot is assumed to be fixed on the ground (its chest)
  bool has_floating_base = false;
  std::string model_pkg = ros::package::getPath("nao_kinect_teleop");
  std::string model_name = model_pkg + "/urdf/naoV40red.urdf";
  // Load the robot model: use RobotModelPin or RobotModelRbdl
  oscr::RobotModel* rmodel = new oscr::RobotModelRbdl(model_name,
                                                     has_floating_base);

  // Get model information: joints and links
  std::vector<std::string> jnames = rmodel->jointNames();
  std::map<std::string, unsigned int> mlink = rmodel->mapLinkNamesIDs();
  // Joint limits (only needed for WQP and HQP)
  Eigen::VectorXd qmin, qmax, dqmax;
  qmin  = rmodel->jointMinAngularLimits();
  qmax  = rmodel->jointMaxAngularLimits();
  dqmax = rmodel->jointVelocityLimits();

  // Initialize ROS
  ros::init(argc, argv, "nao_arms_teleop");
  ros::NodeHandle nh;

  // Initialize interface: oscr control - naoqi
  // It subscribes to "joint_states" and publishes to "joint_angles"
  NaoInterface nao_interface(nh, rmodel->ndofActuated(), jnames);

  // Subscriber to the data of the Kinect v2
  KinectArmPoints kpoints;
  ros::Subscriber sub_1 = nh.subscribe("kinect_points", 1000,
                                       &KinectArmPoints::readKinectPoints,
                                       &kpoints);

  // Get the initial robot configuration (from the "joint_states" topic)
  Eigen::VectorXd qsensed;
  qsensed = nao_interface.getMeasuredJointPositions();
  rmodel->updateJointConfig(qsensed);

  // Inverse Kinematics tasks
  // --------------------------------------------------------------------
  oscr::KineTask* taskrh;
  oscr::KineTask* tasklh;
  oscr::KineTask* taskre;
  oscr::KineTask* taskle;
  taskrh = new oscr::KineTaskPose(rmodel, mlink["r_gripper"], "position");
  taskrh->setGain(200.0);
  // taskrh->setWeight(1.0);
  tasklh = new oscr::KineTaskPose(rmodel, mlink["l_gripper"], "position");
  tasklh->setGain(200.0);
  // tasklh->setWeight(1.0);
  taskre = new oscr::KineTaskPose(rmodel, mlink["RElbow"], "position");
  taskre->setGain(200.0);
  // taskre->setWeight(0.5);
  taskle = new oscr::KineTaskPose(rmodel, mlink["LElbow"], "position");
  taskle->setGain(200.0);
  // taskle->setWeight(0.5);

  // Operational-Space Inverse Kinematics (OSIK) solver
  // --------------------------------------------------------------------
  // Sampling time
  unsigned int f = 200;
  double dt = static_cast<double>(1.0/f);
  // Solver (WQP, HQP or NS)
  oscr::OSIKSolverWQP solver(rmodel, qsensed, dt);
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
    sk_bmarkers.at(i) = new BallMarker(nh, GREEN);
  // Line markers for the human skeleton
  LineMarker sk_lmarkers(nh, GREEN);
  // Ball markers for the retargeted (nao) skeleton
  std::vector<BallMarker*> nao_markers;
  nao_markers.resize(6);
  for (unsigned int i=0; i<nao_markers.size(); ++i)
    nao_markers.at(i) = new BallMarker(nh, LIGHTGRAY, 0.9, 0.03);
  // Line markers for the retargeted (NAO) skeleton
  LineMarker nao_lmarkers(nh, LIGHTGRAY);

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
      // Reset the NAO markers
      nao_lmarkers.reset();
      // NAO lines
      nao_lmarkers.setPose(p_rshoulder); nao_lmarkers.setPose(p_relbow);
      nao_lmarkers.setPose(p_relbow);    nao_lmarkers.setPose(p_rwrist);
      nao_lmarkers.setPose(p_lshoulder); nao_lmarkers.setPose(p_lelbow);
      nao_lmarkers.setPose(p_lelbow);    nao_lmarkers.setPose(p_lwrist);
      nao_lmarkers.publish();

      // Set the desired positions for the tasks
      taskre->setDesiredValue(p_relbow);
      taskrh->setDesiredValue(p_rwrist);
      taskle->setDesiredValue(p_lelbow);
      tasklh->setDesiredValue(p_lwrist);

      // Get the inverse kinematics solution
      solver.getPositionControl(qsensed, qdes);
      rmodel->updateJointConfig(qdes);

      nao_interface.setJointPositionsCmd(qdes);

      // Open or close the hands
      // if (false)
      // {
      //   if(kpoints.getPoints()->left_hand.data==1){
      //     jcmd.joint_angles[13]=1.0;
      //   }
      //   else{
      //     jcmd.joint_angles[13]=0.0;}
      //   if(kpoints.getPoints()->right_hand.data==1){
      //     jcmd.joint_angles[25]=1.0;
      //   }
      //   else{
      //     jcmd.joint_angles[25]=0.0;
      //   }
      // }

      qsensed = qdes;
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
