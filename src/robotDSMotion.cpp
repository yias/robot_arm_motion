#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"


#include <sstream>

#include "LPV.h"
#include "Utils.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>


bool _firstRealPoseReceived=false;

Eigen::VectorXd _eePosition(3);
//Eigen::VectorXd _eeOrientation(4);
Eigen::Matrix3f _wRb;
Eigen::Vector4f _eeOrientation;

bool _firstTargetPoseReceived=false;

Eigen::VectorXd _targetPosition(3);


void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_eePosition << msg->position.x, msg->position.y, msg->position.z;
  	_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_wRb = Utils::quaternionToRotationMatrix(_eeOrientation);
	// Eigen::Vector4d temp;
	// temp=_wRb.cast

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		// _qd = _q;
		//_eePosition = _eePosition+_toolOffset*_wRb.col(2).cast <double> ();
		//_q0 = _q;
		// std::cerr << _qd.transpose() << std::endl;
		
	}
}

void targetListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_targetPosition << msg->position.x, msg->position.y, msg->position.z;
  	//_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
	//_wRb = Utils::quaternionToRotationMatrix(_eeOrientation);
	// Eigen::Vector4d temp;
	// temp=_wRb.cast

	if(!_firstTargetPoseReceived)
	{
		_firstTargetPoseReceived = true;
		// _qd = _q;
		//_eePosition = _eePosition+_toolOffset*_wRb.col(2).cast <double> ();
		//_q0 = _q;
		// std::cerr << _qd.transpose() << std::endl;
		
	}
}




int main(int argc, char **argv)
{

	ros::init(argc, argv, "robotarmdsmotion");

	ros::NodeHandle n;

	ros::Subscriber robotSub=n.subscribe("IIWA/Real_E_Pos", 10, robotListener);

	ros::Subscriber targetSub=n.subscribe("target", 10, targetListener);

	ros::Publisher _pubDesiredPose=n.advertise<geometry_msgs::Pose>("IIWA/Desired_E_Pos", 10);

	geometry_msgs::Pose _msgDesiredPose;

	Eigen::MatrixXd A_tmp;

	Eigen::VectorXd xD(3);

	Eigen::VectorXd desiredNextPosition(3);

	LPV tr_gen;

	tr_gen.initialize(2,3);

	tr_gen.initialize_A("src/robot_arm_motion/models/myModel_fmincon_Amatrix.txt");

	tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_fmincon_priors.txt","src/robot_arm_motion/models/myModel_fmincon_mu.txt","src/robot_arm_motion/models/myModel_fmincon_sigma.txt");

	ros::Rate loop_rate(100);

	int count = 0;

	while (ros::ok()){

		if(_firstTargetPoseReceived){
			A_tmp=tr_gen.Calculate_A(_targetPosition-_eePosition);

			xD=A_tmp*(_targetPosition-_eePosition);

			desiredNextPosition[0]=xD(0)*(1/100)+_eePosition(0);
			desiredNextPosition[1]=xD(1)*(1/100)+_eePosition(1);
			desiredNextPosition[2]=xD(2)*(1/100)+_eePosition(2);

			_msgDesiredPose.position.x=desiredNextPosition[0];
			_msgDesiredPose.position.y=desiredNextPosition[1];
			_msgDesiredPose.position.z=desiredNextPosition[2];
			_msgDesiredPose.orientation.w=_eeOrientation[0];
			_msgDesiredPose.orientation.x=_eeOrientation[1];
			_msgDesiredPose.orientation.y=_eeOrientation[2];
			_msgDesiredPose.orientation.z=_eeOrientation[3];

			_pubDesiredPose.publish(_msgDesiredPose);
	
		}
		
		
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		//chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}