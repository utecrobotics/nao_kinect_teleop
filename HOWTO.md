Teleoperate arms of the real NAO
================================

1. Launch Naoqi locally
   ```
   naoqi 
   ```

2. Launch ROS and NAO bridge replacing NAO_IP with the real IP address of the
   robot (in a new terminal)
   ```
   roslaunch nao_kinect_teleop nao_bringup.launch nao_ip:=NAO_IP
   ```

4. Start sending joint (skeleton) values from the Kinect v2 (in another
   terminal). This needs to be launched from the kinect_v2 package to have
   acces to the NiTE features
   ```
   roscd kinect_v2
   rosrun kinect_v2 mainBodyArray
   ```

5. Wait until the skeleton is properly detected. Then enable the robot motors
   and start the NAO motion (in another terminal)
   ```
   rosservice call /body_stiffness/enable
   rosrun nao_kinect_teleop nao_teleop_arms
   ```

6. To end the motion, first disable the NAO motors
   ```
   rosservice call /body_stiffness/disable 
   ```
Then stop all the previous processes (using Ctrl+C in each terminal)


Teleoperate arms of a simulated NAO (with naoqi)
================================================

1. Launch Naoqi locally
   ```
   naoqi --broker-ip 127.0.0.1
   ```
     
2. Start ROS master, the NAO bridge and Rviz (in a different terminal)
   ```
   roslaunch nao_kinect_teleop nao_bringup.launch nao_ip:=127.0.0.1
   ```

4. Start sending joint (skeleton) values from the Kinect v2 (in another
   terminal)
   ```
   rosrun kinect_v2 mainBodyArray
   ```

5. Wait until the skeleton is properly detected. Then start the NAO motion
   (in another terminal)
   ```
   rosrun nao_kinect_teleop nao_teleop_arms
   ```
