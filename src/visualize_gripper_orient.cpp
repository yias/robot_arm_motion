/**
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  
 *  a ros-node for visualizing the orientation of the gripper
*/

#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>

#include <robot_arm_motion/obstacle_msg.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "Utils.h"


Eigen::VectorXf _eePosition(3); // robot's end-effector position
Eigen::Matrix3f _eeRotMat; // robot's end-effector rotation matrix (according to the frame on the base of the robot)
Eigen::Vector4f _eeOrientation; // robot's end-effector orientation

// velocity orientation
Eigen::Vector3f _eeVelOrient;
float arrow_scale = 0;

// boolean variable for checking if a ee pose is received
bool _isFirstRobotPoseReceived = false;

// boolean variable for checking if a ee desired velocity is received
bool _isFirstDesVelReceived = false;

Eigen::Vector3f tool_translation;
Eigen::Matrix4f tool_transformation_mat;


void robotListener(const geometry_msgs::Pose::ConstPtr& msg)
{
    /***
     *      Callback function for listening the pose of the ee of the robot
     * 
     */

   
    Eigen::Vector4f tmp_ee;
    Eigen::Matrix4f ee_tranformation;
    ee_tranformation = Eigen::Matrix4f::Identity(4, 4);
    tmp_ee << msg->position.x, msg->position.y, msg->position.z, 1.0f;

    _eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    _eeRotMat = Utils<float>::quaternionToRotationMatrix(_eeOrientation);

    ee_tranformation.col(3) = tmp_ee;
    ee_tranformation.block(0, 0, 3, 3) = _eeRotMat;

    Eigen::Matrix4f tt = ee_tranformation * tool_transformation_mat;
    _eePosition = tt.col(3).head(3);

    if (!_isFirstRobotPoseReceived) {
        _isFirstRobotPoseReceived = true;
        ROS_INFO("[robot_arm_motion:orientation visualization] Robot Pose received\n");
    }

}

void velocityListener(const geometry_msgs::Twist::ConstPtr& msg)
{
    /***
     *      Callback function for listening the desired velocity of the ee of the robot
     * 
     */

    Eigen::Vector3f desVel;
    desVel << msg->linear.x, msg->linear.y, msg->linear.z;

    _eeVelOrient = desVel / desVel.norm();

    arrow_scale = desVel.norm();

    if (!_isFirstDesVelReceived) {
        _isFirstDesVelReceived = true;
        ROS_INFO("[robot_arm_motion:orientation visualization] Desired velocity received\n");
    }

}



int main(int argc, char** argv)
{

    // initialize ros node
    ros::init(argc, argv, "obstacles_visualization");

    // define ros node-handler
    ros::NodeHandle n;

    // define ros-subscriber for the robot ee pose
    ros::Subscriber robotSub = n.subscribe("/lwr/ee_pose", 1, robotListener);

    // define ros-subscriber for the robot ee desired velocity
    ros::Subscriber velSub = n.subscribe("/lwr/joint_controllers/passive_ds_command_vel", 1, velocityListener);

    // define ros-subscriber for the robot ee desired velocity
    // ros::Subscriber targetSub = n.subscribe("/target", 1, targetListener);


    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/gripper_orientation", 1);

    ros::Publisher vis_vel_pub = n.advertise<visualization_msgs::Marker>("/vel_orientation", 1);

    


    // set the tool translation from the robot ee to the tip of the tool (or center point the two fingers of the gripper)
    tool_translation << 0.0f, 0.0f, 0.16f;

    tool_transformation_mat = Eigen::Matrix4f::Identity(4, 4);
    Eigen::Matrix3f rel_orient = Eigen::Matrix3f::Zero(3,3);
    rel_orient = Eigen::AngleAxisf(M_PI/2,Eigen::Vector3f::UnitZ());
    tool_transformation_mat.col(3).head(3) = tool_translation;
    tool_transformation_mat.block(0,0,3,3) = rel_orient;

    // define marker message for visualizing the orintation
    visualization_msgs::Marker target_marker;

    target_marker.header.frame_id = "/world";
    target_marker.header.stamp = ros::Time();
    target_marker.ns = "gripper_orient_";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CUBE;
    target_marker.action = visualization_msgs::Marker::ADD;

    // setting the scale, opacity and color of the marker of the target
    target_marker.scale.x = 0.005;
    target_marker.scale.y = 0.12;
    target_marker.scale.z = 0.12;
    target_marker.color.a = 0.9;
    target_marker.color.r = 1.0;
    target_marker.color.g = 1.0;
    target_marker.color.b = 0.0;
    target_marker.lifetime = ros::Duration();

    // define marker message for visualizing the orientation
    visualization_msgs::Marker des_orient;

    des_orient.header.frame_id = "/world";
    des_orient.header.stamp = ros::Time();
    des_orient.ns = "des_orient_";
    des_orient.id = 0;
    des_orient.type = visualization_msgs::Marker::ARROW;
    des_orient.action = visualization_msgs::Marker::ADD;

    // setting the scale, opacity and color of the marker of the target
    des_orient.scale.x = 0.3;
    des_orient.scale.y = 0.01;
    des_orient.scale.z = 0.01;
    des_orient.color.a = 0.9;
    des_orient.color.r = 0.0;
    des_orient.color.g = 0.0;
    des_orient.color.b = 1.0;
    des_orient.lifetime = ros::Duration();


    ros::Rate loop_rate(100);

    while (ros::ok()) {

        // publish the message


        // setting the position and orientation of robot orientation marker
        target_marker.pose.position.x = _eePosition(0);
        target_marker.pose.position.y = _eePosition(1);
        target_marker.pose.position.z = _eePosition(2);

        target_marker.pose.orientation.x = _eeOrientation(1);
        target_marker.pose.orientation.y = _eeOrientation(2);
        target_marker.pose.orientation.z = _eeOrientation(3);
        target_marker.pose.orientation.w = _eeOrientation(0);

        Eigen::Matrix3f vel_orient_mat = Eigen::Matrix3f::Zero(3,3);
        vel_orient_mat = Eigen::AngleAxisf(std::atan2(_eeVelOrient(1), _eeVelOrient(0)),Eigen::Vector3f::UnitZ());
        // vel_orient_mat = Eigen::AngleAxisf(std::atan(_eeVelOrient(1)/ _eeVelOrient(0)),Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f vel_rot_mat_x = Eigen::Matrix3f::Zero(3,3);
        vel_rot_mat_x = Eigen::AngleAxisf(-std::atan2(_eeVelOrient(2), _eeVelOrient(0)),Eigen::Vector3f::UnitY());
        // vel_rot_mat_x = Eigen::AngleAxisf(std::atan(_eeVelOrient(2) / _eeVelOrient(1)),Eigen::Vector3f::UnitY());

        vel_orient_mat = vel_orient_mat*vel_rot_mat_x;

        Eigen::Vector4f vel_orient_q = Utils<float>::rotationMatrixToQuaternion(vel_orient_mat);

        // setting the position and orientation of velocity orientatation marker
        des_orient.pose.position.x = _eePosition(0);
        des_orient.pose.position.y = _eePosition(1);
        des_orient.pose.position.z = _eePosition(2);

        des_orient.pose.orientation.x = vel_orient_q(1);
        des_orient.pose.orientation.y = vel_orient_q(2);
        des_orient.pose.orientation.z = vel_orient_q(3);
        des_orient.pose.orientation.w = vel_orient_q(0);

        des_orient.scale.x = 0.3;


        if (_isFirstRobotPoseReceived) {

            target_marker.header.stamp = ros::Time::now();

            vis_pub.publish(target_marker);
        }

        if (_isFirstDesVelReceived) {

            des_orient.header.stamp = ros::Time::now();

            vis_vel_pub.publish(des_orient);
        }

        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}