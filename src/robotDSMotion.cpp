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

Eigen::VectorXf _eePosition(3);
//Eigen::VectorXd _eeOrientation(4);
Eigen::Matrix3f _eeRotMat;
Eigen::Vector4f _eeOrientation;

bool _firstTargetPoseReceived=false;

Eigen::VectorXf _targetPosition(3);
Eigen::VectorXf _targetOrientation(4);
Eigen::MatrixXf _targetRotMatrix;


void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_eePosition << msg->position.x, msg->position.y, msg->position.z;
	


	if(!_firstRealPoseReceived)
	{
		_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
		_eeRotMat = Utils::quaternionToRotationMatrix(_eeOrientation);
		_firstRealPoseReceived = true;
		ROS_INFO("Robot Pose received\n");
		// _targetPosition[0]=_eePosition[0]-0.1;
		// _targetPosition[1]=_eePosition[1]+0.05;
		// _targetPosition[2]=_eePosition[2]-0.05;
  	
		
	}
}

void targetListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_targetPosition << -msg->position.x, -msg->position.y, msg->position.z;
  	_targetOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_targetRotMatrix = Utils::quaternionToRotationMatrix(_targetOrientation);


	if(!_firstTargetPoseReceived)
	{
		_firstTargetPoseReceived = true;
		ROS_INFO("Target Pose received\n");

		
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

	Eigen::MatrixXf A_tmp;

	Eigen::VectorXf xD(3);

	Eigen::VectorXf desiredNextPosition(3);

	LPV tr_gen;

	tr_gen.initialize(1,3);

	// tr_gen.initialize_A("src/robot_arm_motion/models/myModel_fmincon_Amatrix.txt");
	tr_gen.initialize_A("src/robot_arm_motion/models/myModel_sedumi_Amatrix.txt");

	// tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_fmincon_priors.txt","src/robot_arm_motion/models/myModel_fmincon_mu.txt","src/robot_arm_motion/models/myModel_fmincon_sigma.txt");
	tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_sedumi_priors.txt","src/robot_arm_motion/models/myModel_sedumi_mu.txt","src/robot_arm_motion/models/myModel_sedumi_sigma.txt");

	ros::Rate loop_rate(1000);

	Eigen::MatrixXf W_M(3,3);

	W_M<< -30.0,0.0,0.0,
		  0.0,-30.0,0.0,
		  0.0,0.0,-30.0;

	int count = 0;

	while (ros::ok()){

		if(_firstRealPoseReceived){

			if((_eePosition-_targetPosition).norm()<=0.1f){
				xD=W_M*(_eePosition-_targetPosition);
			}else{
				A_tmp=tr_gen.Calculate_A(_eePosition.cast<double>()-_targetPosition.cast<double>()).cast<float>();

				xD=100.0*A_tmp*(_eePosition-_targetPosition);	
			}
			

			// xD=W_M*(_eePosition-_targetPosition);
			std::cout << "xD=" << xD[0] << " " << xD[1] << " " << xD[2] <<"\n";
			std::cout << "speed: " << xD.norm() << "\n";
			std::cout << "distance: " << (_eePosition-_targetPosition).norm() << "\n";
			std::cout << "target: " << _targetPosition[0] << " " << _targetPosition[1] << " " << _targetPosition[2] <<"\n";

			desiredNextPosition[0]=xD[0]*(1.0/1000.0)+_eePosition(0);
			desiredNextPosition[1]=xD[1]*(1.0/1000.0)+_eePosition(1);
			desiredNextPosition[2]=xD[2]*(1.0/1000.0)+_eePosition(2);
			std::cout << "desiredNextPosition " << desiredNextPosition[0] << " " << desiredNextPosition[1] << " " << desiredNextPosition[2] <<"\n";

			_msgDesiredPose.position.x=desiredNextPosition[0];
			_msgDesiredPose.position.y=desiredNextPosition[1];
			_msgDesiredPose.position.z=desiredNextPosition[2];
			_msgDesiredPose.orientation.w=_eeOrientation[0];
			_msgDesiredPose.orientation.x=_eeOrientation[1];
			_msgDesiredPose.orientation.y=_eeOrientation[2];
			_msgDesiredPose.orientation.z=_eeOrientation[3];

			_pubDesiredPose.publish(_msgDesiredPose);
	
		}
		
		
		// std_msgs::String msg;
		// std::stringstream ss;
		// ss << "hello world " << count;
		// msg.data = ss.str();

		// ROS_INFO("%s", msg.data.c_str());
		//chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}