


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>

#include <robot_arm_motion/obstacle_msg.h>

#include "DSObstacleAvoidance.h"

#include <sstream>

// #include "LPV.h"
#include "Utils.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>


bool _firstRealPoseReceived = false;
bool _firstObstacleReceived = false;
bool _firstTargetPoseReceived = false;

Eigen::VectorXf _eePosition(3);			// robot's end-effector position
Eigen::Matrix3f _eeRotMat;				// robot's end-effector rotation matrix (according to the frame on the base of the robot)
Eigen::Vector4f _eeOrientation;			// robot's end-effector orientation

Eigen::VectorXf _targetPosition(3);		// target position (for the end-effector of the robot)
Eigen::VectorXf _targetOrientation(4);	// target orientation (for the end-effector of the robot)
Eigen::MatrixXf _targetRotMatrix;		// target's rotation matrix (according to the frame on the base of the robot)
Eigen::Vector3f _v;      				// generated end-effector velocity (from the DS) [m/s] (3x1)

std::vector<Obstacle> obstacles;


void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	_eePosition << msg->position.x, msg->position.y, msg->position.z;

	if(!_firstRealPoseReceived)
	{
		_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
		_eeRotMat = Utils<float>::quaternionToRotationMatrix(_eeOrientation);
		_firstRealPoseReceived = true;
		ROS_INFO("[robot_arm_motion:doa] Robot Pose received\n");
	}
}

void updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	_v << msg->linear.x, msg->linear.y, msg->linear.z;
}

void targetListener(const geometry_msgs::Pose::ConstPtr& msg){

	_targetPosition << msg->position.x, msg->position.y, msg->position.z;
  	_targetOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
	_targetRotMatrix = Utils<float>::quaternionToRotationMatrix(_targetOrientation);


	if(!_firstTargetPoseReceived)
	{
		_firstTargetPoseReceived = true;
		ROS_INFO("[robot_arm_motion:doa] Target Pose received\n");
	}
}

void obstacleListener(const robot_arm_motion::obstacle_msg& msg){

	obstacles = std::vector<Obstacle>(msg.obstacles.size());

	for(int i=0; i<msg.obstacles.size(); i++){
		// set the obstacle parameters
		obstacles[i]._x0 << msg.obstacles[i].pose.position.x, msg.obstacles[i].pose.position.y, msg.obstacles[i].pose.position.z; // obstacle position
		obstacles[i]._a << msg.obstacles[i].alpha[0], msg.obstacles[i].alpha[1], msg.obstacles[i].alpha[2]; // obstacle axis lengths
		obstacles[i]._p << msg.obstacles[i].power_terms[0], msg.obstacles[i].power_terms[1], msg.obstacles[i].power_terms[2]; // obstacle power terms
		obstacles[i]._safetyFactor = 1.5; // safety factor
		obstacles[i]._rho = 1.1; // reactivity
		obstacles[i]._tailEffect = false;
		obstacles[i]._bContour = true;
	}

	if(!_firstObstacleReceived)
	{
		_firstObstacleReceived = true;
		ROS_INFO("[robot_arm_motion:doa] Obstacles' poses received\n");
	}
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "obstacle_avoidance");

	ros::NodeHandle n;

	// ros::Subscriber robotSub=n.subscribe("IIWA/Real_E_Pos", 10, robotListener);
	ros::Subscriber robotSub = n.subscribe("/lwr/ee_pose", 10, robotListener);

	ros::Subscriber _subRealTwist = n.subscribe("/lwr/ee_vel", 1, updateRealTwist);

	ros::Subscriber _targetSub=n.subscribe("/target", 10, targetListener);

	ros::Subscriber _obstaclesSub=n.subscribe("/obstacles", 1, obstacleListener);

	ros::Publisher _pubDesiredPose = n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 10);
	ros::Publisher _pubDesiredOrientation = n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	geometry_msgs::Twist _msgDesiredPose;
	geometry_msgs::Quaternion _msgDesiredOrientation;

	visualization_msgs::Marker traj_line;

	traj_line.header.frame_id = "/world";
	traj_line.header.stamp = ros::Time::now();
	traj_line.ns = "obstacle_avoidance";
	traj_line.action = visualization_msgs::Marker::ADD;
	traj_line.pose.orientation.w = 1.0;
	traj_line.id = 0;
	traj_line.type = visualization_msgs::Marker::POINTS;
	traj_line.scale.x = 0.005;
	traj_line.scale.y = 0.005;

	// Points are green
	// traj_line.color.g = 1.0f;
	traj_line.color.b = 1.0f;
	traj_line.color.a = 1.0;


	// Eigen::Vector3d CurrentPos(0);

	Eigen::MatrixXf A_tmp;

	Eigen::VectorXf xD(3);
    Eigen::Vector3f _vd;
    _vd.setConstant(0.0f);

	// Eigen::VectorXf desiredNextPosition(3);

    // define the weight-matrix of the original linear dynamical system
	Eigen::MatrixXf W_M(3,3);
	W_M<< -1.0, 0.0, 0.0,
		  0.0, -1.0, 0.0,
		  0.0, 0.0, -1.0;

	// define ds-modulator for modulating the dynamical system according to the position and size of the obstacles
    DSObstacleAvoidance obsModulator;


	ros::Rate loop_rate(100);

	

	int count = 0;

	while (ros::ok()){

		if(_firstRealPoseReceived){

			xD=W_M*(_eePosition - _targetPosition);

			if (_firstObstacleReceived){
				obsModulator.addObstacles(obstacles);
				_vd = obsModulator.obsModulationEllipsoid(_eePosition, xD, false);
				obsModulator.clearObstacles();
			}else{
				_vd = xD;
			}
            
            
			// Bound desired velocity
			if (_vd.norm()>0.4f)
			{
				_vd = _vd*0.4f/xD.norm();
			}

			// std::cout << "xD:" << xD[0] << " " << xD[1] << " " << xD[2] <<"\n";
            // std::cout << "vd:" << _vd(0) << ", " << _vd(1) << ", " << _vd(2) << std::endl;
			std::cout << "[robot_arm_motion:doa] Speed: " << _vd.norm() << "\n";
			std::cout << "[robot_arm_motion:doa] Distance from target: " << (_eePosition - _targetPosition).norm() << "\n";
			// std::cout << "target: " << _targetPosition[0] << " " << _targetPosition[1] << " " << _targetPosition[2] << std::endl;

			_msgDesiredPose.linear.x=_vd[0];
			_msgDesiredPose.linear.y=_vd[1];
			_msgDesiredPose.linear.z=_vd[2];
			_msgDesiredPose.angular.x = 0.0;
			_msgDesiredPose.angular.y = 0.0;
			_msgDesiredPose.angular.z = 0.0;
			_pubDesiredPose.publish(_msgDesiredPose);

			// Publish desired orientation
			_msgDesiredOrientation.w = _targetOrientation(0);
			_msgDesiredOrientation.x = _targetOrientation(1);
			_msgDesiredOrientation.y = _targetOrientation(2);
			_msgDesiredOrientation.z = _targetOrientation(3);

			_pubDesiredOrientation.publish(_msgDesiredOrientation);

			// update and publish trajectory-visualization points
			geometry_msgs::Point p;
			p.x = _eePosition(0);
			p.y = _eePosition(1);
 			p.z = _eePosition(2);

			traj_line.header.stamp = ros::Time::now();
			traj_line.points.push_back(p);
			marker_pub.publish(traj_line);
			
			
		}
		
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}