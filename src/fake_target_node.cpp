
#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

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


	ros::Rate loop_rate(100);

	int count = 0;

	while (ros::ok()){


        _pubTarget.publish(target_msg);

        obst_msg.header.stamp = ros::Time::now();

        _pubOstacles.publish(obst_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}