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
 */

/*
  -------------------------------------------------------------
  Initial version of the Teleoperation of NAO using a Kinect v2
  -------------------------------------------------------------
*/

#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

#include <oscr/oscr.hpp>

#include <nao_kinect_teleop/deprecated/robotSpecifics.h>
#include <nao_kinect_teleop/deprecated/tools.hpp>
#include <nao_kinect_teleop/kinect-arm-points.hpp>
#include <kinect_msgs/BodyArray.h>


int main(int argc, char **argv)
{
  // Load the urdf model
  // --------------------------------------------------------------------
  // The robot is assumed to be fixed on the ground (its chest)
  bool has_floating_base = false;
  std::string nao_description = ros::package::getPath("nao_description");
  std::string model_name =
    nao_description + "/urdf/naoV40_generated_urdf/nao.urdf";
  oscr::RobotModel* robot = new oscr::RobotModelRbdl(model_name,
                                                     has_floating_base);

  unsigned int ndof_full = robot->ndof();         // All 42 joints
  unsigned int ndof_fingers = 16;
  unsigned int ndof_red = ndof_full-ndof_fingers; // 26 joints (w/o fingers)

  // Get the joint names and joint limits
  std::vector<std::string> jnames = robot->jointNames();
  Eigen::VectorXd qmin, qmax, dqmax;
  qmin  = robot->jointMinAngularLimits();
  qmax  = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();

  //******************************************
  //INICIO DEL PROCESO DE RECEPCION
  //******************************************
  ros::init(argc, argv, "nao_arms_teleop");
  ros::NodeHandle nh;

  KinectArmPoints kpoints;
  //Suscriber
  ros::Subscriber sub_1 = nh.subscribe("kinect_points",
                                       1000,
                                       &KinectArmPoints::readKinectPoints,
                                       &kpoints);
  //Publisher
  ros::Publisher pub
    = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles",
                                                            1000);
  JointSensors jsensor;
  ros::Subscriber sub_2 = nh.subscribe("joint_states",
                                       1000,
                                       &JointSensors::readJointSensors,
                                       &jsensor);

  std::vector< std::vector<double> > P;
  P.resize(6);

  //******************************************
  // Read the initial joint configuration
  // *****************************************

  // Get the initial sensed joint values (from "joint_states" topic)
  std::cout << "Reading initial sensor values ..." << std::endl;
  ros::Rate iter_rate(1000); // Hz
  unsigned int niter = 0, max_iter = 1e3;
  unsigned int ndof_sensed = jsensor.sensedValue()->position.size();
  while (ndof_sensed != ndof_red)
  {
    if (niter++ == max_iter)
    {
      std::cerr << "Initial sensed joint configuration does not have "
                << ndof_red << "degrees of freedom, stopping ..." << std::endl;
      exit(0);
    }
    ndof_sensed = jsensor.sensedValue()->position.size();
    ros::spinOnce();
    iter_rate.sleep();
  }
  std::cout << "Found " << ndof_sensed << " sensed joints" << std::endl;
  // The vector qsensed has 42 dofs (needed for nao full model)
  Eigen::VectorXd qsensed = Eigen::VectorXd::Zero(ndof_full);
  jsensor.getSensedJointsRBDL(qsensed, ndof_sensed);


  // Initialize the joint position control message
  // *********************************************
  naoqi_bridge_msgs::JointAnglesWithSpeed jcmd;
  jcmd.joint_names.resize(ndof_red);
  jcmd.joint_angles.resize(ndof_red);
  jcmd.speed = 0.2;

  // Initialize names of command joints
  for (unsigned int i=0; i<ndof_full; ++i)
  { 
    if (ridx[i] != 100)
     jcmd.joint_names[ridx[i]] = jnames[i];
  }


  // Inverse Kinematics tasks
  // --------------------------------------------------------------------
  oscr::KineTask* taskrh = new oscr::KineTaskPose(robot, RGRIPPER, "position");
  taskrh->setGain(300.0);
  oscr::KineTask* tasklh = new oscr::KineTaskPose(robot, LGRIPPER, "position");
  tasklh->setGain(300.0);
  oscr::KineTask* taskre = new oscr::KineTaskPose(robot, RELBOW, "position");
  taskre->setGain(300.0);
  oscr::KineTask* taskle = new oscr::KineTaskPose(robot, LELBOW, "position");
  taskle->setGain(300.0);

  // Operational-Space Inverse Kinematics (OSIK) solver
  // --------------------------------------------------------------------
  // Sampling time
  unsigned int f = 30;
  double dt = static_cast<double>(1.0/f);
  // Solver (WQP, HQP or NS)
  oscr::OSIKSolverWQP solver(robot, qsensed, dt);
  solver.setJointLimits(qmin, qmax, dqmax);
  // Add tasks to the solver
  solver.pushTask(taskrh);
  solver.pushTask(tasklh);
  solver.pushTask(taskre);
  solver.pushTask(taskle);

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

  //#######################################################
  while(ros::ok())
  {
    unsigned int n_kinect_points = kpoints.getPoints()->body.size();

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

      // Set the desired positions for the tasks
      taskre->setDesiredValue(p_relbow);
      taskrh->setDesiredValue(p_rwrist);
      taskle->setDesiredValue(p_lelbow);
      tasklh->setDesiredValue(p_lwrist);

      solver.getPositionControl(qsensed, qdes);
      jcmd.header.stamp = ros::Time::now();
      reducedJointModel(jcmd.joint_angles, qdes);
      if(kpoints.getPoints()->left_hand.data==1){
	jcmd.joint_angles[13]=1.0;
      }
      else{
	jcmd.joint_angles[13]=0.0;}
      if(kpoints.getPoints()->right_hand.data==1){
	jcmd.joint_angles[25]=1.0;
      }
      else{
	jcmd.joint_angles[25]=0.0;
      }
      pub.publish(jcmd);

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
