#!/usr/bin/env python
"""
    developer: Iason Batzianoulis
    maintaner: Iason Batzianoulis
    email: iasonbatz@gmail.com
    description:
    This scripts launches a socketStream server which receives messages from the gaze client (socketStream client) and publishes the data to a ros-topic
"""

import sys
import argparse
import numpy as np
import numpy.matlib
from scipy.spatial.transform import Rotation as Rot
from socketStream_py import socketStream
import time

import rospy
import geometry_msgs.msg
from std_msgs.msg import Int8
from robot_arm_motion.msg import obstacle_msg
from robot_arm_motion.msg import sobs


class gaze_oRec(object):
    """
        Receive from gaze tracker.
        Send obstacles and target to the robot

    """

    def __init__(self, svrIP='localhost', svrPort=10352, tf_aruco_robot=[0.0, 0.0, 0.0], rot_aruco_robot=[0.0, 0.0, 0.0]):
        # define ros node
        rospy.init_node('gaze_od', anonymous=True)

        # define ros publisher for obstacles
        self.obs_pub = rospy.Publisher('/obstacles',
                                       obstacle_msg, queue_size=1)

        # define ros publisher for obstacles
        self.target_pub = rospy.Publisher('/target',
                                          geometry_msgs.msg.Pose, queue_size=1)

        # define ros subscriber for the robot pose
        self.robot_sub = rospy.Subscriber('/lwr/ee_pose',
                                          geometry_msgs.msg.Pose, self.get_robot_pose)

        # define fake-target subscriber for debugging
        self.fake_target_sub = rospy.Subscriber(
            '/fake_target', Int8, self.get_fake_target)

        # set initial values for robot position and orientation (wxyz)
        self.robot_ee_Position = np.array([0.0, 0.0, 0.0])
        self.robot_ee_Orientation = np.array([0.0, 0.0, 0.0, 0.0])

        # set initial value for the fake target
        self.fake_target = -1

        # define alpha and power term for the obstacles (fixed for all the obstacles)
        self.obs_alpha = [0.1, 0.2, 0.1]
        self.obs_power_term = [1.0, 1.0, 1.0]

        # define tranformation from aruco-board frame to robot-frame (only position)
        self.tf_aruco_robot = np.array(tf_aruco_robot)
        self.rot_mat = self.euler2rotMat(rot_aruco_robot)
        self.tf_mat = self.compTF(rot_aruco_robot, tf_aruco_robot)

        # define the obstacles' z-coordinate (fixed for all the obstacles)
        self.obs_z = 0.2

        # define the obstacles' orientation in quaternions - wxyz (fixed for all the obstacles)
        self.obs_orient = [0.0, 0.0, 1.0, 0.0]

        # define the target's z-coordinate
        self.target_z = 0.3

        # define the target's orientation in quaternions - wxyz
        self.target_orient = [0.0, 0.0, 1.0, 0.0]

        # define a socketStream server to handle the communication
        self.sockHndlr = socketStream.socketStream(
            svrIP=svrIP, svrPort=svrPort, socketStreamMode=1)

    def get_robot_pose(self, msg):
        """
        Callback function for receiving the robot pose

        """
        self.robot_ee_Position = np.array(
            [msg.position.x, msg.position.y, msg.position.z])
        self.robot_ee_Orientation = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

    def get_fake_target(self, msg):
        """
        Callback function for receiving the fake target

        """
        self.fake_target = msg.data

    def euler2rotMat(self, eulerAngles):
        """
        compute the rotation matrix from euler angles

        """
        # convertion to radians
        eulerAngles = np.array(eulerAngles) * np.pi / 180.0

        # computing the rotation matrix
        rr = np.array([[np.cos(eulerAngles[0]) * np.cos(eulerAngles[1]), np.cos(eulerAngles[0]) * np.sin(eulerAngles[1]) * np.sin(eulerAngles[2]) - (np.sin(eulerAngles[0]) * np.cos(eulerAngles[2])), np.cos(eulerAngles[0]) * np.sin(eulerAngles[1]) * np.cos(eulerAngles[2]) + (np.sin(eulerAngles[0]) * np.sin(eulerAngles[2]))],
                       [np.sin(eulerAngles[0]) * np.cos(eulerAngles[1]), np.sin(eulerAngles[0]) * np.sin(eulerAngles[1]) * np.sin(eulerAngles[2]) + (np.cos(eulerAngles[0]) *
                                                                                                                                                     np.cos(eulerAngles[2])), np.sin(eulerAngles[0]) * np.sin(eulerAngles[1]) * np.cos(eulerAngles[2]) - (np.cos(eulerAngles[0]) * np.sin(eulerAngles[2]))],
                       [-np.sin(eulerAngles[1]), np.cos(eulerAngles[1]) * np.sin(eulerAngles[2]), np.cos(eulerAngles[1]) * np.cos(eulerAngles[2])]])

        return rr

    def compTF(self, eulerAngles, position):
        """
        compute the transformation matrix from euler angles and position

        """
        rotMat = self.euler2rotMat(eulerAngles)
        tf_mat = np.hstack([rotMat, np.array(position).reshape(3, 1)])
        tf_mat = np.vstack([tf_mat, np.array([0.0, 0.0, 0.0, 1.0])])
        return(tf_mat)

    def run(self):
        """
        running the node

        """
        # initialize socketStream server and launch it
        self.everything_ok = False
        if self.sockHndlr.initialize_socketStream() == 0:
            if self.sockHndlr.runServer() == 0:
                self.everything_ok = True

        if self.everything_ok:
            while(True):
                try:
                    if self.sockHndlr.socketStream_ok():
                        tt = self.sockHndlr.get_latest()
                        # print(tt)
                        if tt is not None:

                            # retrieve the obstacles from the socketStream message
                            obsData = tt['obj_location']
                            obj_locations = np.array(obsData, dtype=np.float32)
                            # get the number of obstacles
                            nb_obstacles = obj_locations.shape[0]
                            print("nb obstacles: ", nb_obstacles)

                            # if at least one obstacle is found
                            if nb_obstacles != 0:

                                # convert from cm to meters and append a column for the z-coordinate
                                obj_locations = np.hstack(
                                    [obj_locations / 100, np.ones((nb_obstacles, 1))*self.obs_z])
                                # transform obstacles to the robot-frame
                                obj_locations = np.hstack([
                                    obj_locations, np.ones((nb_obstacles, 1))])
                                obj_locations = np.transpose(
                                    np.dot(self.tf_mat, np.transpose(obj_locations)))

                                # define the message to be published
                                obs_msg = obstacle_msg()

                                # set the properties of the obstacles in the message
                                for i in range(nb_obstacles):
                                    # if a fake target is received, exclude the corresponding obstacle to be set as target
                                    if i != self.fake_target:
                                        tmp_obstacle = sobs()
                                        tmp_obstacle.alpha = self.obs_alpha
                                        tmp_obstacle.power_terms = self.obs_power_term

                                        tmp_obstacle.pose.position.x = obj_locations[i, 0]
                                        tmp_obstacle.pose.position.y = obj_locations[i, 1]
                                        tmp_obstacle.pose.position.z = obj_locations[i, 2]

                                        tmp_obstacle.pose.orientation.w = self.obs_orient[0]
                                        tmp_obstacle.pose.orientation.x = self.obs_orient[1]
                                        tmp_obstacle.pose.orientation.y = self.obs_orient[2]
                                        tmp_obstacle.pose.orientation.z = self.obs_orient[3]
                                        obs_msg.obstacles.append(tmp_obstacle)

                                # add the time stamp to the header of the obs_msg
                                now = rospy.get_rostime()
                                obs_msg.header.stamp.secs = now.secs
                                obs_msg.header.stamp.nsecs = now.nsecs
                                # publish the message
                                self.obs_pub.publish(obs_msg)

                            # retrieve the  object of interest from message (defined as the target)
                            targetData = tt['oboi']
                            target_location = np.array(
                                targetData, dtype=np.float32)

                            # convert from cm to meters and append a column for the z-coordinate
                            target_location = np.hstack(
                                [target_location / 100, self.target_z])
                            # transform target to the robot-frame
                            target_location = np.hstack(
                                [target_location, 1.0])
                            target_location = np.dot(
                                self.tf_mat, target_location.reshape(4, 1))

                            # if the received coordinates for the object of interest are not zero,
                            # set the object of interest as target and publish the message
                            if target_location[0] != 0 and target_location[0] != 0:
                                target_msg = geometry_msgs.msg.Pose()
                                target_msg.position.x = target_location[0]
                                target_msg.position.y = target_location[1]
                                target_msg.position.z = target_location[2]
                                target_msg.orientation.w = self.target_orient[0]
                                target_msg.orientation.x = self.target_orient[1]
                                target_msg.orientation.y = self.target_orient[2]
                                target_msg.orientation.z = self.target_orient[3]
                                self.target_pub.publish(target_msg)

                            # if a fake target is received, set the corresponding obstacle as target (for debugging)
                            if self.fake_target >= 0 and self.fake_target < nb_obstacles:
                                target_msg = geometry_msgs.msg.Pose()
                                target_msg.position.x = obj_locations[self.fake_target, 0]
                                target_msg.position.y = obj_locations[self.fake_target, 1]
                                target_msg.position.z = obj_locations[self.fake_target, 2]
                                target_msg.orientation.w = self.target_orient[0]
                                target_msg.orientation.x = self.target_orient[1]
                                target_msg.orientation.y = self.target_orient[2]
                                target_msg.orientation.z = self.target_orient[3]
                                self.target_pub.publish(target_msg)

                    if rospy.is_shutdown():
                        break
                except KeyboardInterrupt:
                    # with Cntl+C, close any socket communication
                    self.sockHndlr.closeCommunication()
                    break

        self.sockHndlr.closeCommunication()


if __name__ == '__main__':

    # get server IP and port
    svrIP = rospy.get_param("/ss_serverIP")
    svrPort = rospy.get_param("/ss_serverPort")

    # get the translation from aruco-board frame to robot-frame
    tf_arc_robot = rospy.get_param("/aruco_position")

    # get the rotation of the robot-frame to the frame of the aruco-board
    rot_arc_robot = rospy.get_param("/frame_rotations")

    # define a gaze_oRec object
    eRes_handler = gaze_oRec(
        svrIP=svrIP, svrPort=svrPort, tf_aruco_robot=tf_arc_robot, rot_aruco_robot=rot_arc_robot)

    # run the node
    eRes_handler.run()
