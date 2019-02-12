#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "LPV.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "robotarmdsmotion");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


	LPV tr_gen;

	tr_gen.initialize(2,3);

	tr_gen.initialize_A("src/robot_arm_motion/models/myModel_fmincon_Amatrix.txt");

	tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_fmincon_priors.txt","src/robot_arm_motion/models/myModel_fmincon_mu.txt","src/robot_arm_motion/models/myModel_fmincon_sigma.txt");

	ros::Rate loop_rate(10);

	int count = 0;

	while (ros::ok()){
		
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}