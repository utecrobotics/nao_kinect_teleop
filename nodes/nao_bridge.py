#!/usr/bin/env python

#                                                                               
# Copyright 2016
# J.Avalos, S.Cortez, O.Ramos. 
# Universidad de Ingenieria y Tecnologia - UTEC
#
# This file is part of nao_kinect_teleop.
# nao_kinect_teleop is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
# nao_kinect_teleop is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details. You should
# have received a copy of the GNU Lesser General Public License along
# with nao_kinect_teleop. If not, see <http://www.gnu.org/licenses/>.
#


import rospy

from naoqi_driver.naoqi_node import NaoqiNode
from sensor_msgs.msg import JointState

from naoqi_bridge_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse, SetArmsEnabled, SetArmsEnabledResponse


class NaoMotionBridge(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_motion_bridge')

        self.connectNaoQi()
        self.moveTest()
        rospy.Subscriber("joint_position_cmd", JointState, self.sendJointsToNAO)

    def connectNaoQi(self):
        """(re-) connect to NaoQI
        """
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def sendJointsToNAO(self, jointCmd):
        jointNames = jointCmd.name
        jointValues = list(jointCmd.position)
        fractionMaxSpeed = 0.2
        self.motionProxy.setAngles(jointNames, jointValues, fractionMaxSpeed)

    def moveTest(self):
        names  = ["HeadYaw", "HeadPitch"]
        angles  = [0.5, -0.5]
        fractionMaxSpeed  = 0.2
        self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
        print "Head yaw and pitch set to: ", angles


if __name__ == '__main__':
    motionBridge = NaoMotionBridge()
    rospy.loginfo("nao_motion_bridge running ...")
    rospy.spin()
    rospy.loginfo("nao_motion_bridge stopping ...")
    exit(0)
