#include "header.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <kinect_msgs/BodyArray.h>
#include <geometry_msgs\Twist.h>
#include <windows.h>
#include <Kinect.h>


using std::string;


int main()
{
  int orden[6] = { 4, 5, 7, 8, 9, 11 };

  //Inicializacion del Kinect
  IKinectSensor* pSensor;
  HRESULT hResult = S_OK;
  hResult = GetDefaultKinectSensor(&pSensor);
  hResult = pSensor->Open();
  
  IBodyFrameSource* pBodySource;
  hResult = pSensor->get_BodyFrameSource(&pBodySource);
  
  IBodyFrameReader* pBodyReader;
  hResult = pBodySource->OpenReader(&pBodyReader);
  
  ICoordinateMapper* pCoordinateMapper;
  hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
  
  //Inicializacion del envio de datos (Ubuntu)
  ros::NodeHandle nh;
  char *ros_master = "10.100.144.116";

  printf("Connecting to server at %s\n", ros_master);
  nh.initNode(ros_master);

  printf("Advertising kinect_points message\n");
  kinect_msgs::BodyArray body_msg;
  body_msg.body = new geometry_msgs::Vector3[6];
  body_msg.body_length = 6;
  body_msg.left_hand.data = false;
  body_msg.right_hand.data = false;

  ros::Publisher kinect_points_pub("kinect_points", &body_msg);
  nh.advertise(kinect_points_pub);

  printf("Go robot go!\n");
  while (1)
  {
    Sleep(100);
    
    IBodyFrame* pBodyFrame = nullptr;
    hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
    if (SUCCEEDED(hResult)){
      IBody* pBody[BODY_COUNT] = { 0 };
      hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
      
      if (pBodyFrame != nullptr)
      {
        for (int count = 0; count < BODY_COUNT; count++)
        {
          BOOLEAN bTracked = false;
          hResult = pBody[count]->get_IsTracked(&bTracked);
          
          if (SUCCEEDED(hResult) && bTracked)
          {
            Joint joint[JointType::JointType_Count];
            hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
            
            if (SUCCEEDED(hResult))
            {
              // Estado de mano izquierda
              HandState leftHandState = HandState::HandState_Unknown;
              hResult = pBody[count]->get_HandLeftState(&leftHandState);
              
              if (SUCCEEDED(hResult)){
                if (leftHandState == HandState::HandState_Open){
                  body_msg.left_hand.data = true;
                }
                if (leftHandState == HandState::HandState_Closed){
                  body_msg.left_hand.data = false;
                }
              }
              
              // Estado de mano derecha
              HandState rightHandState = HandState::HandState_Unknown;
              hResult = pBody[count]->get_HandRightState(&rightHandState);
              
              if (SUCCEEDED(hResult)){
                if (rightHandState == HandState::HandState_Open){
                  body_msg.right_hand.data = true;
                }
                if (rightHandState == HandState::HandState_Closed){
                  body_msg.right_hand.data = false;
                }
              }
              
              //Articulaciones
              for (int i = 0; i < 6; i++){
                if (joint[orden[i]].TrackingState != TrackingState::TrackingState_NotTracked){
                  CameraSpacePoint cameraSpacePoint = joint[orden[i]].Position;
                  
                  body_msg.body[i].x = joint[orden[i]].Position.X;
                  body_msg.body[i].y = joint[orden[i]].Position.Y;
                  body_msg.body[i].z = joint[orden[i]].Position.Z;
                }
              }
              kinect_points_pub.publish(&body_msg);
              nh.spinOnce();
            }
          }
        }
      }
      SafeRelease(pBodyFrame);
    }
  }
  SafeRelease(pBodySource);
  SafeRelease(pBodyReader);
  SafeRelease(pCoordinateMapper);
  if (pSensor)
    pSensor->Close();

  SafeRelease(pSensor);
  return 0;
  printf("All done!\n");
  return 0;
}
