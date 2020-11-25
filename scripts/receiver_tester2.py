#!/usr/bin/env python
"""
    developer: Iason Batzianoulis
    maintaner: Iason Batzianoulis
    email: iasonbatz@gmail.com
    description: 
    This scripts launches a socketStream server which receives messages from the gaze client (socketStream client) and publishes the data to a ros-topic
"""

import argparse
import numpy as np
import sys
from socketStream_py import socketStream
import time

import rospy
import geometry_msgs.msg
from robot_arm_motion.msg import obstacle_msg
from robot_arm_motion.msg import sobs


def main(args):

    # define ros node
    rospy.init_node('gaze_od', anonymous=True)

    # define ros publisher for obstacles
    obs_pub = rospy.Publisher('/obstacles',
                              obstacle_msg, queue_size=1)

    # define ros publisher for obstacles
    target_pub = rospy.Publisher('/target',
                                 geometry_msgs.msg.Pose, queue_size=1)

    # define alpha and power term for the obstacles (fixed for all the obstacles)
    obs_alpha = [0.1, 0.2, 0.1]
    obs_power_term = [1.0, 1.0, 1.0]

    # define the obstacles' z-coordinate (fixed for all the obstacles)
    obs_z = 0.2

    # define the obstacles' orientation in quaternions - wxyz (fixed for all the obstacles)
    obs_orient = [0.0, 0.0, 1.0, 0.0]

    # define the target's z-coordinate
    target_z = 0.3

    # define the target's orientation in quaternions - wxyz
    target_orient = [0.0, 0.0, 1.0, 0.0]

    # define a socketStream server to handle the communication
    sockHndlr = socketStream.socketStream(
        svrIP=args.host, svrPort=args.port, socketStreamMode=1)

    # initialize socketStream server and launch it
    everything_ok = False
    if sockHndlr.initialize_socketStream() == 0:
        if sockHndlr.runServer() == 0:
            everything_ok = True

    if everything_ok:
        while(True):
            try:
                if sockHndlr.socketStream_ok():
                    tt = sockHndlr.get_latest()
                    # print(tt)
                    if tt is not None:

                        # retrieve the data from the message
                        obsData = tt['obj_location']
                        obj_locations = np.array(obsData, dtype=np.float32)
                        targetData = tt['oboi']
                        target_location = np.array(
                            targetData, dtype=np.float32)

                        nb_obstacles = obj_locations.shape[0]
                        print("nb objects: ", nb_obstacles)

                        # define the message to be published
                        obs_msg = obstacle_msg()

                        # set the properties of the obstacles in the message
                        for i in range(nb_obstacles):
                            tmp_obstacle = sobs()
                            tmp_obstacle.alpha = obs_alpha
                            tmp_obstacle.power_terms = obs_power_term

                            # convert to meters
                            tmp_obstacle.pose.position.x = obj_locations[i, 0] / 10
                            # convert to meters
                            tmp_obstacle.pose.position.y = obj_locations[i, 1] / 10
                            tmp_obstacle.pose.position.z = obs_z

                            tmp_obstacle.pose.orientation.w = obs_orient[0]
                            tmp_obstacle.pose.orientation.x = obs_orient[1]
                            tmp_obstacle.pose.orientation.y = obs_orient[2]
                            tmp_obstacle.pose.orientation.z = obs_orient[3]
                            obs_msg.obstacles.append(tmp_obstacle)

                        now = rospy.get_rostime()
                        obs_msg.header.stamp.secs = now.secs
                        obs_msg.header.stamp.nsecs = now.nsecs
                        obs_pub.publish(obs_msg)

                        if target_location[0] != 0 and target_location[0] != 0:
                            target_msg = geometry_msgs.msg.Pose()
                            target_msg.position.x = target_location[0]
                            target_msg.position.y = target_location[1]
                            target_msg.position.z = target_z
                            target_msg.orientation.w = target_orient[0]
                            target_msg.orientation.x = target_orient[1]
                            target_msg.orientation.y = target_orient[2]
                            target_msg.orientation.z = target_orient[3]
                            target_pub.publish(target_msg)

                if rospy.is_shutdown():
                    break
            except KeyboardInterrupt:
                # with Cntl+C, close any socket communication
                sockHndlr.closeCommunication()
                break

    sockHndlr.closeCommunication()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='TCP server for receiving inputs from a client with socketStream')
    parser.add_argument('--host', type=str,
                        help='the IP of the server', default='128.178.145.15')
    parser.add_argument(
        '--port', type=int, help='the port on which the server is listening', default=10353)
    parser.add_argument('--buffersize', type=int,
                        help='the size of the buffer for pakets receiving', default=64)
    args = parser.parse_args()
    main(args)
