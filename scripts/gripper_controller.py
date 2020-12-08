#!/usr/bin/env python
"""
    developer: Iason Batzianoulis
    maintaner: Iason Batzianoulis
    email: iasonbatz@gmail.com
    description:
    This scripts listens to commands from a ros-topic and commands the Robotiq 85 2-finger gripper accordingly
"""

import sys
import argparse
import numpy as np
import numpy.matlib
import time

import rospy
import geometry_msgs.msg
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


class gripper_controller(object):
    """
        Class for opening or closing the gripper

    """

    def __init__(self):
        # define ros node
        rospy.init_node('gripper_controller', anonymous=True)

        # define ros publisher for sending command to the gripper interfaces
        self.grip_pub = rospy.Publisher(
            '/gripper/Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

        # define ros publisher for sending joint states
        self.jointState_pub = rospy.Publisher(
            '/gripper/joint_states', JointState, queue_size=1)

        # define a subscriber for listening to the gripper position
        self.gripper_listener = rospy.Subscriber(
            "/gripper/Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.gripperListener)

        # define ros subscriber for the robot pose
        self.grip_sub = rospy.Subscriber('/gripper/command',
                                         Int8, self.command_receiver)

        # define a variable to hold the position of the gripper
        self.gripper_position = 0.0

        # define a JointState message for the gripper joint states
        self.js_msg = JointState()
        self.js_msg.name = ["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint",
                            "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]

        # self.js_msg.position.resize(6)

        self.command = outputMsg.Robotiq2FGripper_robot_output()

        self.current_command = 0

        self.gripper_initialized = False

    def command_receiver(self, msg):
        """
            Callback function for receiving the command for the gripper

        """
        self.current_command = msg.data

    def gripperListener(self, msg):
        """
            Callback function for listening the position of the gripper

        """
        self.gripper_position = msg.gPO

    def publishCommand(self):
        """
            function for publishing the command
        """

        if not self.gripper_initialized:
            self.command = outputMsg.Robotiq2FGripper_robot_output()
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP = 255
            self.command.rFR = 150
            self.gripper_initialized = True
        else:
            if self.current_command == 0:
                # reinitialize gripper
                self.gripper_initialized = True
            if self.current_command == 1:
                # close the gripper
                self.command.rPR = 255
            if self.current_command == 2:
                # open the gripper
                self.command.rPR = 0

        self.grip_pub.publish(self.command)

    def publishGripperJointAngles(self):
        self.js_msg.position = [self.gripper_position/255.0, self.gripper_position / 255.0, -self.gripper_position /
                                255.0, self.gripper_position / 255.0, self.gripper_position / 255.0, -self.gripper_position / 255.0]
        # add the time stamp to the header of the obs_msg
        now = rospy.get_rostime()
        self.js_msg.header.stamp.secs = now.secs
        self.js_msg.header.stamp.nsecs = now.nsecs
        self.jointState_pub.publish(self.js_msg)

    def run(self):
        """
            running the node

        """
        while(True):
            try:
                self.publishCommand()
                self.publishGripperJointAngles()
                rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break
            except KeyboardInterrupt:
                break


if __name__ == '__main__':

    # define a gripper_controller object
    gripper_handler = gripper_controller()

    # run the node
    gripper_handler.run()
