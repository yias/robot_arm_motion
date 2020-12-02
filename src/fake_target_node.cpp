
#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"

#include <robot_arm_motion/obstacle_msg.h>



int main(int argc, char **argv)
{

    // initialize ros node
	ros::init(argc, argv, "fake_target_publisher");

    // define ros node-handler
	ros::NodeHandle n;
	
    // define ros-publisher for the target
	ros::Publisher _pubTarget = n.advertise<geometry_msgs::Pose>("/target", 1);

    // define ros-publisher for the obstacles
    ros::Publisher _pubOstacles = n.advertise<robot_arm_motion::obstacle_msg>("/obstacles", 1);

    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    // define ros-message for the obstacles
    robot_arm_motion::obstacle_msg obst_msg;

    // define 2 obstacles

    obst_msg.obstacles.resize(2);

    // define the position and orienation of the first obstacle
    obst_msg.obstacles[0].pose.position.x = -0.2f;
    obst_msg.obstacles[0].pose.position.y = -0.4f;
    obst_msg.obstacles[0].pose.position.z = 0.2f;

    obst_msg.obstacles[0].pose.orientation.w = 0.0f;
    obst_msg.obstacles[0].pose.orientation.x = 0.0f;
    obst_msg.obstacles[0].pose.orientation.y = 1.0f;
    obst_msg.obstacles[0].pose.orientation.z = 0.0f;

    // define the axis lengths and the power terms of the first obstacle
    obst_msg.obstacles[0].alpha[0] = 0.1f;
    obst_msg.obstacles[0].alpha[1] = 0.2f;
    obst_msg.obstacles[0].alpha[2] = 0.1f;

    obst_msg.obstacles[0].power_terms[0] = 1.0f;
    obst_msg.obstacles[0].power_terms[1] = 1.0f;
    obst_msg.obstacles[0].power_terms[2] = 1.0f;


    // for visualizing the first obstacle
    visualization_msgs::Marker obs_marker0;
    obs_marker0.header.frame_id = "/world";
    obs_marker0.header.stamp = ros::Time();
    obs_marker0.ns = "obstacle_";
    obs_marker0.id = 0;
    obs_marker0.type = visualization_msgs::Marker::SPHERE;
    obs_marker0.action = visualization_msgs::Marker::ADD;
    obs_marker0.pose.position.x = obst_msg.obstacles[0].pose.position.x;
    obs_marker0.pose.position.y = obst_msg.obstacles[0].pose.position.y;
    obs_marker0.pose.position.z = obst_msg.obstacles[0].pose.position.z;
    obs_marker0.pose.orientation.x = obst_msg.obstacles[0].pose.orientation.x;
    obs_marker0.pose.orientation.y = obst_msg.obstacles[0].pose.orientation.y;
    obs_marker0.pose.orientation.z = obst_msg.obstacles[0].pose.orientation.z;
    obs_marker0.pose.orientation.w = obst_msg.obstacles[0].pose.orientation.w;
    obs_marker0.scale.x = obst_msg.obstacles[0].alpha[0];
    obs_marker0.scale.y = obst_msg.obstacles[0].alpha[1];
    obs_marker0.scale.z = obst_msg.obstacles[0].alpha[2];
    obs_marker0.color.a = 0.7;
    obs_marker0.color.r = 0.0;
    obs_marker0.color.g = 1.0;
    obs_marker0.color.b = 0.0;
    obs_marker0.lifetime = ros::Duration();


    // similarly define the position, orientation, alpha and power terms for the second obstacle

    obst_msg.obstacles[1].pose.position.x = 0.2f;
    obst_msg.obstacles[1].pose.position.y = -0.4f;
    obst_msg.obstacles[1].pose.position.z = 0.2f;

    obst_msg.obstacles[1].pose.orientation.w = 0.0f;
    obst_msg.obstacles[1].pose.orientation.x = 0.0f;
    obst_msg.obstacles[1].pose.orientation.y = 1.0f;
    obst_msg.obstacles[1].pose.orientation.z = 0.0f;


    obst_msg.obstacles[1].alpha[0] = 0.1f;
    obst_msg.obstacles[1].alpha[1] = 0.1f;
    obst_msg.obstacles[1].alpha[2] = 0.3f;

    obst_msg.obstacles[1].power_terms[0] = 1.0f;
    obst_msg.obstacles[1].power_terms[1] = 1.0f;
    obst_msg.obstacles[1].power_terms[2] = 1.0f;


    // for visualizing the second obstacle
    visualization_msgs::Marker obs_marker1;
    obs_marker1.header.frame_id = "/world";
    obs_marker1.header.stamp = ros::Time();
    obs_marker1.ns = "obstacle_";
    obs_marker1.id = 1;
    obs_marker1.type = visualization_msgs::Marker::SPHERE;
    obs_marker1.action = visualization_msgs::Marker::ADD;
    obs_marker1.pose.position.x = obst_msg.obstacles[1].pose.position.x;
    obs_marker1.pose.position.y = obst_msg.obstacles[1].pose.position.y;
    obs_marker1.pose.position.z = obst_msg.obstacles[1].pose.position.z;
    obs_marker1.pose.orientation.x = obst_msg.obstacles[1].pose.orientation.x;
    obs_marker1.pose.orientation.y = obst_msg.obstacles[1].pose.orientation.y;
    obs_marker1.pose.orientation.z = obst_msg.obstacles[1].pose.orientation.z;
    obs_marker1.pose.orientation.w = obst_msg.obstacles[1].pose.orientation.w;
    obs_marker1.scale.x = obst_msg.obstacles[1].alpha[0];
    obs_marker1.scale.y = obst_msg.obstacles[1].alpha[1];
    obs_marker1.scale.z = obst_msg.obstacles[1].alpha[2];
    obs_marker1.color.a = 0.7;
    obs_marker1.color.r = 0.0;
    obs_marker1.color.g = 1.0;
    obs_marker1.color.b = 0.0;
    obs_marker1.lifetime = ros::Duration();


    // define the message for the target
    geometry_msgs::Pose target_msg;

    // define the position and orienation for the target
    target_msg.position.x = 0.5;
    target_msg.position.y = -0.4;
    target_msg.position.z = 0.22;

    target_msg.orientation.w = 0.0; 
    target_msg.orientation.x = 0.0;
    target_msg.orientation.y = 1.0;
    target_msg.orientation.z = 0.0;


    // for visualizing the second obstacle
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "/world";
    target_marker.header.stamp = ros::Time();
    target_marker.ns = "target_marker_";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose.position.x = target_msg.position.x;
    target_marker.pose.position.y = target_msg.position.y;
    target_marker.pose.position.z = target_msg.position.z;
    target_marker.pose.orientation.x = target_msg.orientation.x;
    target_marker.pose.orientation.y = target_msg.orientation.y;
    target_marker.pose.orientation.z = target_msg.orientation.z;
    target_marker.pose.orientation.w = target_msg.orientation.w;
    target_marker.scale.x = 0.07;
    target_marker.scale.y = 0.07;
    target_marker.scale.z = 0.01;
    target_marker.color.a = 0.9;
    target_marker.color.r = 0.0;
    target_marker.color.g = 0.4;
    target_marker.color.b = 0.9;
    target_marker.lifetime = ros::Duration();


	ros::Rate loop_rate(100);

	int count = 0;

	while (ros::ok()){


        _pubTarget.publish(target_msg);

        obst_msg.header.stamp = ros::Time::now();

        _pubOstacles.publish(obst_msg);

        obs_marker0.header.stamp = ros::Time::now();

        vis_pub.publish(obs_marker0);

        obs_marker1.header.stamp = ros::Time::now();

        vis_pub.publish(obs_marker1);

        target_marker.header.stamp = ros::Time::now();

        vis_pub.publish(target_marker);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}