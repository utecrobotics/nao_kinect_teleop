//
// Teleoperation of NAO using a Kinect v2
//
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <robot-model/robot-model.hpp>
#include <osik-control/math-tools.hpp>
#include <osik-control/kine-task.hpp>
#include <osik-control/kine-task-pose.hpp>
#include <osik-control/kine-solver-WQP.hpp>
#include <nao_kinect_teleop/robotSpecifics.h>
#include <nao_kinect_teleop/tools.hpp>
#include <kinect_msgs/BodyArray.h>
#include <cmath>

using namespace osik;


//################ CLASS #################################
class KinectPoints
{
public:
  KinectPoints()
    : msg_(new kinect_msgs::BodyArray)
  {
     std::cout << msg_->body.size() << std::endl;
  }
  void readKinectPoints(const kinect_msgs::BodyArray::ConstPtr& msg)
  {
    msg_ = msg;
  }
  kinect_msgs::BodyArray::ConstPtr getPoints()
  {
    return msg_;
  }
private:
  kinect_msgs::BodyArray::ConstPtr msg_;
};
//########################################################


int main(int argc, char **argv)
{
  //INICIO DEL PROCESO DE INFORMACION AL NAO
  std::string nao_description = ros::package::getPath("nao_description");
  std::string model_name = nao_description + "/urdf/naoV40_generated_urdf/nao.urdf";
  RobotModel* robot = new RobotModel();
  bool has_floating_base = false;
  if (!robot->loadURDF(model_name, has_floating_base))
    return -1;
  else
    std::cout << "Robot " << model_name << " loaded." << std::endl;

  unsigned int ndof_full = robot->ndof();        // All 42 joints
  unsigned int ndof_fingers = 16;
  unsigned int ndof_red = ndof_full-ndof_fingers; // 26 joints (excluding fingers)

  // Get the joint names and joint limits
  std::vector<std::string> jnames;
  std::vector<double> qmin, qmax, dqmax;
  jnames = robot->jointNames();
  qmin  = robot->jointMinAngularLimits();
  qmax  = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();
  //******************************************
  //INICIO DEL PROCESO DE RECEPCION
  //******************************************
  ros::init(argc, argv, "show_points");
  ros::NodeHandle nh;

  KinectPoints kpoints;
  //Suscriber
  ros::Subscriber sub_1 = nh.subscribe("kinect_points", 1000, &KinectPoints::readKinectPoints, &kpoints);
  //Publisher
  ros::Publisher pub = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 1000);
  JointSensors jsensor;
  ros::Subscriber sub_2 = nh.subscribe("joint_states", 1000,&JointSensors::readJointSensors, &jsensor);

  std::vector< std::vector<double> > P;
  P.resize(6);

  //******************************************
  // Read the initial joint configuration
  // *****************************************

  // Get the initial sensed joint values (from "joint_states" topic)
  std::cout << "Reading initial sensor values ..." << std::endl;
  ros::Rate iter_rate(1000); // Hz
  unsigned int niter=0, max_iter = 1e3;
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

  // Tasks and Inverse Kinematics Solver
  // ************************************

  // Sampling time
  unsigned int f = 30;   // Frequency
  double dt = static_cast<double>(1.0/f);

  KineSolverWQP solver(robot, qsensed, dt);
  solver.setJointLimits(qmin, qmax, dqmax);

 
  //KineTask* taskrh = new KineTaskPose(robot, RGRIPPER, "position");
  KineTask* taskrh = new KineTaskPose(robot, RGRIPPER, "position");
  taskrh->setGain(300.0);
  KineTask* tasklh = new KineTaskPose(robot, LGRIPPER, "position");
  tasklh->setGain(300.0);
  KineTask* taskre = new KineTaskPose(robot, RELBOW, "position");
  taskre->setGain(300.0);
  KineTask* taskle = new KineTaskPose(robot, LELBOW, "position");
  taskle->setGain(300.0);

  Eigen::VectorXd P_right_wrist;
  Eigen::VectorXd P_right_elbow;
  Eigen::VectorXd P_left_wrist;
  Eigen::VectorXd P_left_elbow;
  
  solver.pushTask(taskrh);
  solver.pushTask(tasklh);
  solver.pushTask(taskre);
  solver.pushTask(taskle);

  Eigen::VectorXd qdes;

  ros::Rate rate(f); // Hz

  //#######################################################
  while(ros::ok())
  {
    std::cout << "size: " << kpoints.getPoints()->body.size() << std::endl;

    if (kpoints.getPoints()->body.size() > 0)
    {
      //Datos del Nao
      double L1 = 0.108; //Del hombro al codo
      double L2 = 0.111; //Del codo a la mano
      
      //Recepcion del brazo derecho
      for (unsigned k=0;k<(P.size()/2);k++)
      {    
        P[k].resize(3);
        //A partir de eso P[0][k]=(0,0,0)
        P[k][0] = (-kpoints.getPoints()->body[k].z)-(-kpoints.getPoints()->body[0].z);
        P[k][1] = (-kpoints.getPoints()->body[k].x)-(-kpoints.getPoints()->body[0].x);
        P[k][2] = (kpoints.getPoints()->body[k].y)-(kpoints.getPoints()->body[0].y);
      }

      //Construimos los puntos (0,0,0); P1; P2
      //Hallamos el modulo M1 de P1
      double M1= sqrt(pow(P[1][0],2.0)+pow(P[1][1],2.0)+pow(P[1][2],2.0));
      //Hallamos el modulo M2 de (P2-P1)
      double M2= sqrt(pow(P[2][0]- P[1][0], 2.0) + pow(P[2][1]- P[1][1], 2.0) +
                      pow(P[2][2]- P[1][2], 2.0));
      //Determimos las proporcionalidades
      double Q1 = L1 / M1;
      double Q2 = L2 / M2;

      //Redefinimos P1
      P[0][0] = 0.00;
      P[0][1] = 0.098;
      P[0][2] = 0.100;
      //Redefinimos P2
      P[2][0] = Q2*(P[2][0] - P[1][0]);
      P[2][1] = Q2*(P[2][1] - P[1][1]);
      P[2][2] = Q2*(P[2][2] - P[1][2]);
      //Redefinimos P1
      P[1][0] = P[0][0]+Q1*P[1][0];
      P[1][1] = P[0][1]+Q1*P[1][1];
      P[1][2] = P[0][2]+Q1*P[1][2];

      P[2][0] = P[2][0]+P[1][0];
      P[2][1] = P[2][1]+P[1][1];
      P[2][2] = P[2][2]+P[1][2];

      // Recepcion del brazo izquierdo
      for (unsigned k=(P.size()/2);k<P.size();k++)
      {    
	P[k].resize(3);
	P[k][0] = (-kpoints.getPoints()->body[k].z)-(-kpoints.getPoints()->body[3].z);
	P[k][1] = (-kpoints.getPoints()->body[k].x)-(-kpoints.getPoints()->body[3].x);
	P[k][2] = (kpoints.getPoints()->body[k].y)-(kpoints.getPoints()->body[3].y);
      } 
      //Hallamos el modulo M1 de P4
      double M3 = sqrt(pow(P[4][0], 2.0) + pow(P[4][1], 2.0) + pow(P[4][2], 2.0));
      //Hallamos el modulo M2 de (P2-P1)
      double M4 = sqrt(pow(P[5][0] - P[4][0], 2.0) + pow(P[5][1] - P[4][1], 2.0) + pow(P[5][2] - P[4][2], 2.0));

      double Q3 = L1 / M3;
      double Q4 = L2 / M4;
      //Construimos los puntos (0,0,0); P4; P5
      P[3][0] = 0.00;
      P[3][1] = -0.098;
      P[3][2] = 0.100;
      //Redefinimos P5
      P[5][0] = Q4*(P[5][0] - P[4][0]);
      P[5][1] = Q4*(P[5][1] - P[4][1]);
      P[5][2] = Q4*(P[5][2] - P[4][2]);
      //Redfinimos P4
      P[4][0] = P[3][0]+Q3*P[4][0];
      P[4][1] = P[3][1]+Q3*P[4][1];
      P[4][2] = P[3][2]+Q3*P[4][2];

      P[5][0] = P[5][0]+P[4][0];
      P[5][1] = P[5][1]+P[4][1];
      P[5][2] = P[5][2]+P[4][2];
      //#######################################################

      P_right_wrist.resize(3);
      P_left_wrist.resize(3);
      P_right_elbow.resize(3);
      P_left_elbow.resize(3);
      
      //Elbow Izquierdo
      P_left_elbow[0] = P[1][0];
      P_left_elbow[1] = P[1][1];
      P_left_elbow[2] = P[1][2];
        
      //Left hand
      P_left_wrist[0] = P[2][0];
      P_left_wrist[1] = P[2][1];
      P_left_wrist[2] = P[2][2];
        
      //Right elbow
      P_right_elbow[0] = P[4][0];
      P_right_elbow[1] = P[4][1];
      P_right_elbow[2] = P[4][2];
        
      //Right hand
      P_right_wrist[0] = P[5][0];
      P_right_wrist[1] = P[5][1];
      P_right_wrist[2] = P[5][2];
        
      taskle->setDesiredValue(P_left_elbow);
      tasklh->setDesiredValue(P_left_wrist);
      taskre->setDesiredValue(P_right_elbow);
      taskrh->setDesiredValue(P_right_wrist);
    
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
    ros::spinOnce();
    rate.sleep();
    
  }
  //#######################################################3
  return 0;
}
