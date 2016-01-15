Example of Usage with NAO
=========================

1. Launch Naoqi locally
     ./naoqi 

2. Launch ROS and NAO bridge
     roslaunch nao_bringup nao_full_py.launch nao_ip:=NAO_IP

3. Run RVIZ (optional)
     roslaunch nao_osik display.launch

4. Connect both computers (after this activation, the computer running
   windows can send the Kinect information to the ROS Master)
     rosrun rosserial_server socket_node

5. Enable NAO motors
     rosservice call /body_stiffness/enable

6. Send the motion to NAO
     rosrun nao_kinect_teleop nao_arms_teleop

To end the motion, stop ros and disable the NAO motors:
     rosservice call /body_stiffness/disable 

Notes:
 - NAO_IP is the NAO IP address 
 - After (4), it might be useful to check the incoming data from the Kinect:
     rostopic echo /kinect_points
