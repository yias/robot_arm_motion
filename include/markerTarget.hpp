/***
 *      a node for listening to optitrack markers (rigid bodies)
 *      and define the position and orienation of the target 
 *      with respect to the robot
 *      
 *      the end-effector of the robot will follow the target marker
 *      employing a linear DS, with the same gains on the components
 *      of its velocity (check robotDSMotion for that)
 * 
 * 
 */

#ifndef __MARKER_TARGET_HPP__
#define __MARKER_TARGET_HPP__

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include <dynamic_reconfigure/server.h>
#include <robot_arm_motion/targetOffsetsConfig.h>

#include <signal.h>
#include <mutex>
#include <iostream>
#include <sstream>

// #include "LPV.h"
#include "Utils.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>


class markerTarget {


    // class-related variables
    // Eigen::VectorXf _eePosition(3);         // position of the end-efffetor of the robot
    // Eigen::Vector4f _eeOrientation;         // orientation of the end-effector of the robot
    // Eigen::Matrix3f _eeRotMat;              // rotation matrix for the end-effector of the robot

    Eigen::Vector3f _targetPosition;     // position of the target with respect to the base of the robot
    Eigen::Vector4f _targetOrientation;     // orientation of the target with respect to the base of the robot(?)
    Eigen::Matrix3f _targetRotMatrix;       // rotation matrix of the target
    Eigen::Matrix3f _rotMatrix;
    double alpha;

    Eigen::Vector3f _mkrPosition;        // position of the marker in the world frame
    Eigen::Vector4f _mkrOrientation;        // orientation of the marker in the world frame
    Eigen::Matrix3f _mkrRotMat;             // rotation matrix for the marker

    Eigen::Vector3f _rbtPosition;        // position of the robot-base in the world frame
    Eigen::Vector4f _rbtOrientation;        // orientation of the robot-base in the world frame
    Eigen::Matrix3f _rbtRotMat;             // rotation matrix for the robot-base
    
    Eigen::Vector3f _vcurr;      			// Current end effector velocity [m/s] (3x1)

    Eigen::Vector3f _targetOffsets;         // offsets of the marker on the x, y and z direction (the offset indicates how far the target will be defined with respect to the robot)



    bool _firstMarkerPoseReceived=false;    // boolean flag to check if the first marker pose is received
    double _mkrTimeStamp=0;                 // the time-stamp of the marker's subscriber

    bool _firstRbtBasePoseReceived=false;   // boolean flag to check if the first robot's base pose is received
    double _rbtTimeStamp=0;                 // the time-stamp of the robot's base subscriber

    // ros-related variables

    ros::NodeHandle _n;                     // ros node-handler
    ros::Rate _loopRate;                    // the loop-rate of ros spin

    // subscribers
    ros::Subscriber _mkrSub;                // subscriber to listen to the pose of the marker
    ros::Subscriber _robotBaseSub;          // subscriber to listen to the pose of the robot base

    // publishers
    ros::Publisher _pubtarget;              // publiser for streaming the desired target pose

    // messages
    geometry_msgs::Pose _tagetPoseMsg;      // the message to be published

    double _initTime;                       // ros time when the node started
    bool _stop;                             // Monitor CTRL+C event

    // mutex for controlling the message publishing
    std::mutex _mutex;                      // Mutex variable

    static markerTarget* me;                // Pointer on the instance

    // methods

    int computeDesiredTarget();             // computing the deisred target pose

    // Publish data to topics
    void publishData();

    // Callback to update the robot's base pose
    void updateRobotBasePose(const geometry_msgs::PoseStamped& msg); 

    // Callback function to update the marker's pose
    void updateMkrPose(const geometry_msgs::PoseStamped& msg); 

    // Stop node callback 
	static void stopNodeCallback(int sig);

    // callback function for dynamic reconfigurable offsets
    void dynRecCallback(robot_arm_motion::targetOffsetsConfig &config, uint32_t level);

    dynamic_reconfigure::Server<robot_arm_motion::targetOffsetsConfig> dynServer;
    dynamic_reconfigure::Server<robot_arm_motion::targetOffsetsConfig>::CallbackType _f;


public:
    // constructor
    markerTarget(ros::NodeHandle &n, double frequency);

    bool init();                            // node initialization

    int run();                              // run node main loop


};

#endif