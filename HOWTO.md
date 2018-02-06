Teleoperate arms and head of the real NAO
=========================================

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
   acces to the NiTE features:
   ```
   roscd kinect_v2
   rosrun kinect_v2 mainBodyArray
   ```

5. Wait until the skeleton is properly detected. Then enable the robot motors
   (in another terminal)
   ```
   rosservice call /body_stiffness/enable
   ```

6. Start the NAO motion (in another terminal)
   ```
   rosrun nao_kinect_teleop nao_teleop_arms_head
   ```

**Ending the motion**: To end the motion, first disable the NAO motors *twice*
(or more times, if necessary, until the motors do not have any sound)
   ```
   rosservice call /body_stiffness/disable 
   rosservice call /body_stiffness/disable 
   ```
Then stop all the previous processes (using Ctrl+C in each terminal). There
is currently an issue with thread running the kinect process and it needs to
be manually killed. To this end, go to the terminal used in step 4 and press
Ctrl+C; then press Ctrl+Z; then execute
   ```
   kill %
   jobs
   ```
There should be a message saying: [Number]+ Terminated.


Teleoperate a simulated NAO (with naoqi)
================================================

The following teleoperates the arms and head of the NAO (as in the previous case)
but, instead of using the real robot, it uses the *simulation* provided by naoqi.

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
   roscd kinect_v2
   rosrun kinect_v2 mainBodyArray
   ```

5. Wait until the skeleton is properly detected. Then start the NAO motion
   (in another terminal)
   ```
   rosrun nao_kinect_teleop nao_teleop_arms_head
   ```
**Ending the motion**: To end the motionstop all the previous processes 
(using Ctrl+C in each terminal). There is currently an issue with thread
running the kinect process and it needs to be manually killed as mentioned
before.
