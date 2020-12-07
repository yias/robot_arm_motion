/**
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  
 *  a ros-node for listeining to obstacles and targets and publishing the corresponding visualization markers for rviz
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

// define marker message for visualizing the target
visualization_msgs::Marker target_marker;

// define vector of marker messages for the visualization of the obstacles
std::vector<visualization_msgs::Marker> obs_markers;

// boolean variable for checking if a target message is received
bool isTargetMsgReceived = false;

void targetListener(const geometry_msgs::Pose ::ConstPtr& tgtMsg)
{
    /***
     *      Callback function for listening the target coordinates and update the visualization marker
     * 
     */

    target_marker.header.frame_id = "/world";
    target_marker.header.stamp = ros::Time();
    target_marker.ns = "target_marker_";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.action = visualization_msgs::Marker::ADD;

    // setting the position and orientation of target the marker
    target_marker.pose.position.x = tgtMsg->position.x;
    target_marker.pose.position.y = tgtMsg->position.y;
    target_marker.pose.position.z = tgtMsg->position.z;

    target_marker.pose.orientation.x = tgtMsg->orientation.x;
    target_marker.pose.orientation.y = tgtMsg->orientation.y;
    target_marker.pose.orientation.z = tgtMsg->orientation.z;
    target_marker.pose.orientation.w = tgtMsg->orientation.w;

    // setting the scale, opacity and color of the marker of the target
    target_marker.scale.x = 0.07;
    target_marker.scale.y = 0.07;
    target_marker.scale.z = 0.01;
    target_marker.color.a = 0.9;
    target_marker.color.r = 0.0;
    target_marker.color.g = 0.4;
    target_marker.color.b = 0.9;
    target_marker.lifetime = ros::Duration();

    if (!isTargetMsgReceived) {
        isTargetMsgReceived = true;
    }
}

void obstaclesListener(const robot_arm_motion::obstacle_msg& obsMsg)
{
    /***
     *      Callback function for listening the obstacles' coordinates and update the visualization markers
     * 
     */

    size_t nb_obstacles = obsMsg.obstacles.size();

    if (obs_markers.size() < nb_obstacles) {
        obs_markers = std::vector<visualization_msgs::Marker>(nb_obstacles);
    }

    for (size_t i = 0; i < nb_obstacles; i++) {
        obs_markers[i].header.frame_id = "/world";
        obs_markers[i].header.stamp = ros::Time();
        obs_markers[i].ns = "obstacle_"; // setting the name-space
        obs_markers[i].id = i; // setting the id of the marker
        obs_markers[i].type = visualization_msgs::Marker::SPHERE; // setting the shape of the marker
        obs_markers[i].action = visualization_msgs::Marker::ADD; // add this marker to the existing ones

        // setting the position and orientation of target the obstacle
        obs_markers[i].pose.position.x = obsMsg.obstacles[i].pose.position.x;
        obs_markers[i].pose.position.y = obsMsg.obstacles[i].pose.position.y;
        obs_markers[i].pose.position.z = obsMsg.obstacles[i].pose.position.z;

        obs_markers[i].pose.orientation.x = obsMsg.obstacles[i].pose.orientation.x;
        obs_markers[i].pose.orientation.y = obsMsg.obstacles[i].pose.orientation.y;
        obs_markers[i].pose.orientation.z = obsMsg.obstacles[i].pose.orientation.z;
        obs_markers[i].pose.orientation.w = obsMsg.obstacles[i].pose.orientation.w;

        // setting the scale of the marker depending on the alpha parameters osf the obstacle
        obs_markers[i].scale.x = obsMsg.obstacles[i].alpha[0];
        obs_markers[i].scale.y = obsMsg.obstacles[i].alpha[1];
        obs_markers[i].scale.z = obsMsg.obstacles[i].alpha[2];

        // setting the color (rbg) and the opacity (a) of the obstacle
        obs_markers[i].color.a = 0.7;
        obs_markers[i].color.r = 0.0;
        obs_markers[i].color.g = 1.0;
        obs_markers[i].color.b = 0.0;
        obs_markers[i].lifetime = ros::Duration();
    }

    // delete the remaining markers
    for (size_t i = nb_obstacles; i < obs_markers.size(); i++) {
        obs_markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

int main(int argc, char** argv)
{

    // initialize ros node
    ros::init(argc, argv, "fake_target_publisher");

    // define ros node-handler
    ros::NodeHandle n;

    // define ros-subscriber for the target
    ros::Subscriber _subTarget = n.subscribe("/target", 1, targetListener);

    // define ros-subscriber for the obstacles
    ros::Subscriber _subOstacles = n.subscribe("/obstacles", 1, obstaclesListener);

    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/obstacles_visualization", 10);

    // // define ros-message for the obstacles
    // robot_arm_motion::obstacle_msg obst_msg;

    // // define the message for the target
    // geometry_msgs::Pose target_msg;

    // // fill the messeges with the parameters of the obstacles
    // if (parse_obs_params("ram_obstacles_position", "ram_obstacles_orientation", "ram_obstacles_alpha", "ram_obstacles_power_params", &obs_marker_msgs, &obst_msg) < 0) {
    //     std::cout << "[obstacle-target simulation] Correct the obstacles parameters in yaml file and launch again the node" << std::endl;
    //     return -1;
    // }

    // // fill the messeges with the parameters of the target
    // if (parse_target_params("ram_target_position", "ram_target_orientation", &target_marker, &target_msg) < 0) {
    //     std::cout << "[obstacle-target simulation] Correct the obstacles parameters in yaml file and launch again the node" << std::endl;
    //     return -2;
    // }

    ros::Rate loop_rate(100);

    while (ros::ok()) {

        // publish the messages

        for (size_t i = 0; i < obs_markers.size(); i++) {
            obs_markers[i].header.stamp = ros::Time::now();
            vis_pub.publish(obs_markers[i]);
        }

        if (isTargetMsgReceived) {

            target_marker.header.stamp = ros::Time::now();

            vis_pub.publish(target_marker);

            isTargetMsgReceived = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
        // ++count;
    }

    return 0;
}