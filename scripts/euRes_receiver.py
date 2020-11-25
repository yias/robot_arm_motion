
# import ros-related modules
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import sys

# import socketStream to handl the socket communication
import socketStream

# import numpy for easier data manipulation
import numpy as np


def talker():
    # define ros publisher
    pub = rospy.Publisher('socketStreamer/MultiArray',
                          Float32MultiArray, queue_size=2)

    # define ros node
    rospy.init_node('socketStream_node', anonymous=True)

    # set the frequncy to 100Hz
    rate = rospy.Rate(100)

    # define the message to be published as a float32 (double) multi-array
    rosmsg = Float32MultiArray()
    rosmsg.layout.data_offset = 0

    # since it is a 2D matrix, append two times a MultiArrayDimension object
    rosmsg.layout.dim.append(MultiArrayDimension())
    rosmsg.layout.dim.append(MultiArrayDimension())

    # define a socketStram object to handle the communication, setting the IP address of the machine that this program is running and the port to be open.
    # the port should be the same on the client side too
    sockHndlr = socketStream.socketStream(
        IPaddress="localhost", port=10352, bufferSize=16)

    while not rospy.is_shutdown():
        if sockHndlr.sockectStream_ok():
            # get the latest message from socketStream
            socket_msg = sockHndlr.get_latest()

            # get the message data
            socket_data = socket_msg.get("data")

            # throw the data to a numpy ndarray  and get its dimensions
            data_matrix = np.array(socket_data, dtype=np.float32)
            data_dimensions = list(data_matrix.shape)

            # construct the message
            rosmsg.layout.dim[0].label = socket_msg.get("name")
            rosmsg.layout.dim[1].label = socket_msg.get("name")

            rosmsg.layout.dim[0].size = data_dimensions[0]
            rosmsg.layout.dim[0].stride = np.prod(data_dimensions)

            rosmsg.layout.dim[1].size = data_dimensions[1]
            rosmsg.layout.dim[1].stride = data_dimensions[1]

            rosmsg.data = data_matrix.reshape(np.prod(data_dimensions))

            # publish the message
            pub.publish(rosmsg)

    sockHndlr.close_communication()


if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
