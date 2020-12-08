#!/usr/bin/env python
"""
    developer: Iason Batzianoulis
    maintaner: Iason Batzianoulis
    email: iasonbatz@gmail.com
    description:
    This scripts simulates the opening and closing of the Robotiq 85 2-finger gripper
"""

import sys
import numpy as np
import numpy.matlib
# from scipy.spatial.transform import Rotation as Rot
import time

import rospy
import geometry_msgs.msg
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


class gripper_sim(object):
    def __init__(self):
        # define ros node
        rospy.init_node('gripper_sim', anonymous=True)

        self.grip_pub = rospy.Publisher(
            '/gripper/Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=1)

        self.grip_sub = rospy.Subscriber(
            "/gripper/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, self.gripperListener)

        self.current_position = 0.0

        self.required_position = 0.0

        self.step = 1.0

        self.pos_upper_limit = 210.0

        self.pos_lower_limit = 0.0

    def gripperListener(self, msg):
        self.required_position = msg.rPR

    def publishMsg(self):
        if self.required_position > self.current_position:
            self.current_position += self.step
            if self.current_position > self.pos_upper_limit:
                self.current_position = self.pos_upper_limit
        if self.required_position < self.current_position:
            self.current_position -= self.step
            if self.current_position < self.pos_lower_limit:
                self.current_position = self.pos_lower_limit
        msg = inputMsg.Robotiq2FGripper_robot_input()
        msg.gPO = int(self.current_position)
        self.grip_pub.publish(msg)

    def run(self):
        while(True):
            try:
                self.publishMsg()
                rospy.sleep(0.01)
                if rospy.is_shutdown():
                    break
            except KeyboardInterrupt:
                break


if __name__ == '__main__':

    # define a gripper_sim object
    gripper_handler = gripper_sim()

    # run the node
    gripper_handler.run()
