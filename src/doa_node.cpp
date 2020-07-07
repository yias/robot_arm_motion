


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "DSObstacleAvoidance.h"

// #include "DynamicObstacleAvoidance/Modulation.hpp"
// #include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
// #include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
// // #include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
// #include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
// #include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
// #include "DynamicObstacleAvoidance/Agent.hpp"


#include <sstream>

// #include "LPV.h"
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
Eigen::Vector3f _v;      				// Current end effector velocity [m/s] (3x1)


void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_eePosition << msg->position.x, msg->position.y, msg->position.z;
	


	if(!_firstRealPoseReceived)
	{
		_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
		_eeRotMat = Utils<float>::quaternionToRotationMatrix(_eeOrientation);
		_firstRealPoseReceived = true;
		ROS_INFO("Robot Pose received\n");
		// _targetPosition[0]=_eePosition[0]-0.1;
		// _targetPosition[1]=_eePosition[1]+0.05;
		// _targetPosition[2]=_eePosition[2]-0.05;
  	
		
	}
}

void updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	_v << msg->linear.x, msg->linear.y, msg->linear.z;
}

void targetListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_targetPosition << msg->position.x, msg->position.y, msg->position.z;
  	_targetOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_targetRotMatrix = Utils<float>::quaternionToRotationMatrix(_targetOrientation);


	if(!_firstTargetPoseReceived)
	{
		_firstTargetPoseReceived = true;
		ROS_INFO("Target Pose received\n");

		
	}
}




int main(int argc, char **argv)
{

	ros::init(argc, argv, "obstacleavoidance");

	ros::NodeHandle n;

	// ros::Subscriber robotSub=n.subscribe("IIWA/Real_E_Pos", 10, robotListener);
	ros::Subscriber robotSub = n.subscribe("/lwr/ee_pose", 10, robotListener);

	ros::Subscriber _subRealTwist = n.subscribe("/lwr/ee_vel", 1, updateRealTwist);

	ros::Subscriber targetSub=n.subscribe("target", 10, targetListener);

	ros::Publisher _pubDesiredPose=n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 10);
	ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

	geometry_msgs::Twist _msgDesiredPose;
	geometry_msgs::Quaternion _msgDesiredOrientation;

	Eigen::MatrixXf A_tmp;

	Eigen::VectorXf xD(3);
    Eigen::Vector3f _vd;
    _vd.setConstant(0.0f);

	Eigen::VectorXf desiredNextPosition(3);

    Obstacle _obs1, _obs2;
    DSObstacleAvoidance obsModulator;


    // define the first obstacle
    _obs1._x0 << -0.5f, 0.01f, 0.2f; // obstacle position
    _obs1._a << 0.5f,0.1f,0.12f;
    _obs1._p.setConstant(1.0f);
    _obs1._safetyFactor = 2.0;
	_obs1._rho = 3.0;
    _obs1._tailEffect = false;
	_obs1._bContour = false;


    obsModulator.addOstacle(_obs1);

    // define the second obstacle
    _obs2._x0 << 0.95f, 0.85f, 0.90f; 
    _obs2._a << 0.5f,0.1f,0.12f;
    _obs2._p.setConstant(1.0f);
    _obs2._safetyFactor = 1.0;
	_obs2._rho = 3.0;
    _obs2._tailEffect = false;
	_obs2._bContour = false;


	// LPV tr_gen;

	// tr_gen.initialize(1,3);

	// // tr_gen.initialize_A("src/robot_arm_motion/models/myModel_fmincon_Amatrix.txt");
	// tr_gen.initialize_A("src/robot_arm_motion/models/myModel_sedumi_Amatrix.txt");

	// // tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_fmincon_priors.txt","src/robot_arm_motion/models/myModel_fmincon_mu.txt","src/robot_arm_motion/models/myModel_fmincon_sigma.txt");
	// tr_gen.initialize_theta("src/robot_arm_motion/models/myModel_sedumi_priors.txt","src/robot_arm_motion/models/myModel_sedumi_mu.txt","src/robot_arm_motion/models/myModel_sedumi_sigma.txt");

	ros::Rate loop_rate(100);

	Eigen::MatrixXf W_M(3,3);

	W_M<< -1.0,0.0,0.0,
		  0.0,-1.0,0.0,
		  0.0,0.0,-1.0;

	int count = 0;

	while (ros::ok()){

		if(_firstRealPoseReceived){

			xD=W_M*(_eePosition - _targetPosition);

			// if((_eePosition-_targetPosition).norm()<=0.1f){
			// 	xD=W_M*(_eePosition-_targetPosition);
			// }else{
			// 	// A_tmp=tr_gen.Calculate_A(_eePosition.cast<double>()-_targetPosition.cast<double>()).cast<float>();

			// 	xD=100.0*A_tmp*(_eePosition-_targetPosition);	
			// }
            
            _vd = obsModulator.obsModulationEllipsoid(_eePosition, xD, false);
			// Bound desired velocity
			if (_vd.norm()>0.3f)
			{
				xD = xD*0.3f/xD.norm();
			}
			

			// xD=W_M*(_eePosition-_targetPosition);
			std::cout << "xD=" << xD[0] << " " << xD[1] << " " << xD[2] <<"\n";
            std::cout << "vd: " << _vd(0) << ", " << _vd(1) << ", " << _vd(2) << std::endl;
			std::cout << "speed: " << _vd.norm() << "\n";
			std::cout << "distance: " << (_eePosition-_targetPosition).norm() << "\n";
			std::cout << "target: " << _targetPosition[0] << " " << _targetPosition[1] << " " << _targetPosition[2] << std::endl;

            

            

			// desiredNextPosition[0]=xD[0]*(1.0/1000.0)+_eePosition(0);
			// desiredNextPosition[1]=xD[1]*(1.0/1000.0)+_eePosition(1);
			// desiredNextPosition[2]=xD[2]*(1.0/1000.0)+_eePosition(2);
			// std::cout << "desiredNextPosition " << desiredNextPosition[0] << " " << desiredNextPosition[1] << " " << desiredNextPosition[2] <<"\n";

			// _msgDesiredPose.position.x=desiredNextPosition[0];
			// _msgDesiredPose.position.y=desiredNextPosition[1];
			// _msgDesiredPose.position.z=desiredNextPosition[2];
			// _msgDesiredPose.orientation.w=_eeOrientation[0];
			// _msgDesiredPose.orientation.x=_eeOrientation[1];
			// _msgDesiredPose.orientation.y=_eeOrientation[2];
			// _msgDesiredPose.orientation.z=_eeOrientation[3];

			_msgDesiredPose.linear.x=_vd[0];
			_msgDesiredPose.linear.y=_vd[1];
			_msgDesiredPose.linear.z=_vd[2];
			_msgDesiredPose.angular.x = 0.0;
			_msgDesiredPose.angular.y = 0.0;
			_msgDesiredPose.angular.z = 0.0;
			_pubDesiredPose.publish(_msgDesiredPose);

			// _msgDesiredPose.orientation.w=_targetOrientation[0];
			// _msgDesiredPose.orientation.x=_targetOrientation[1];
			// _msgDesiredPose.orientation.y=_targetOrientation[2];
			// _msgDesiredPose.orientation.z=_targetOrientation[3];

			// Publish desired orientation
			_msgDesiredOrientation.w = _targetOrientation(0);
			_msgDesiredOrientation.x = _targetOrientation(1);
			_msgDesiredOrientation.y = _targetOrientation(2);
			_msgDesiredOrientation.z = _targetOrientation(3);

			_pubDesiredOrientation.publish(_msgDesiredOrientation);

			
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