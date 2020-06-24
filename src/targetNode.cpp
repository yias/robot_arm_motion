#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>

#include "Utils.h"
// #include "gaze_tracking/coordinate.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h> 



// // ----------------- variables for the robot base -----------------

// bool _firstKukaBasePoseReceived=false;
// double robotBaseStamp=0;

// Eigen::VectorXf robot_base_position(3); 	// the position of the robot's base as received from the mocap system
// Eigen::VectorXf robot_base_orienation(4);	// the orienation of the robot's base
// Eigen::Matrix3f robot_base_rotMatrix;		// the rotation matrix of the orientation of the robot's base 



// // ----------------- variables for the object 1 -----------------

// bool _firstObjectPoseReceived=false;
// double objectPoseStamp=0;
// float _objectZOffset=0.1f;
// float _objectYOffset=0.0f;

// Eigen::Vector3f obj1_To_Robot;

// Eigen::VectorXf object_position(3);									// the position of the object as received from the mocap system
// Eigen::VectorXf object_orientation(4);								// the orientation of the object as received from the mocap system
// Eigen::Matrix3f object_rotMatrix;


// // ----------------- variables for the object 2 -----------------

// bool _firstObject2PoseReceived=false;
// double object2PoseStamp=0;
// float _object2ZOffset=0.1f;
// float _object2YOffset=0.0f;

// Eigen::Vector3f obj2_To_Robot;

// Eigen::VectorXf object2_position(3);								// the position of the object 2 as received from the mocap system
// Eigen::VectorXf object2_orientation(4);								// the orientation of the object 2 as received from the mocap system
// Eigen::Matrix3f object2_rotMatrix;


// // ----------------- variables for the ARCU-board frame -----------------

// bool _firstARCUPoseReceived=false;
// double ARCUboard_PoseStamp=0;
// float _ARCUboard_ZOffset=0.01f;
// float _ARCUboard_YOffset=0.025f;

// Eigen::VectorXf ARCUboard_position(3);								// the position of the frame on the ARCU-borad as received from the mocap system
// Eigen::VectorXf ARCUboard_orientation(4);							// the orientation of the frame on the ARCU-borad as received from the mocap system
// Eigen::Matrix3f ARCUboard_rotMatrix;


// // ----------------- variables for the target -----------------

// bool _TargetPoseReceived=false;
// float _Target_XOffset=0.0f;
// float _Target_YOffset=0.0f;
// float _Target_ZOffset=0.0f;

// Eigen::VectorXf ARCU_Target_position(2);							// the position of the target on the ARCU-borad as received from the gaze tracker
// Eigen::Vector4f gt_msg_coord;
// bool gt_flag, validity_flag=false;
// double classif_threshold=0.1;




// // ----------------- callback function for the base listener -----------------

// void robotBaseListener(const geometry_msgs::PoseStamped& msg){

// 	robot_base_position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
//   	robot_base_orienation << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
// 	robot_base_rotMatrix = Utils::quaternionToRotationMatrix(robot_base_orienation);


// 	if(!_firstKukaBasePoseReceived)
// 	{
// 		ROS_INFO("Initial robot base pose received\n");
// 		_firstKukaBasePoseReceived = true;
// 		robotBaseStamp=msg.header.seq;
		
		
// 	}
// }










int main(int argc, char **argv){

	// initialize the node
    ros::init(argc, argv, "tagetnode");

	ros::NodeHandle n;

	ros::Publisher _pubtarget=n.advertise<geometry_msgs::Pose>("target", 10);

	geometry_msgs::Pose _msgTargetPose;

	Eigen::VectorXf targetPosition(3);

	Eigen::VectorXf targetOrientation(4);

	ros::Rate loop_rate(100);

	
    targetPosition[0] = -0.5;
    targetPosition[1] = 0.6;
    targetPosition[2] = 0.2;

    targetOrientation[0] = 0.0;
    targetOrientation[1] = 0.0;
    targetOrientation[2] = 1.0;
    targetOrientation[3] = 0.0;

	int count = 0;

	while (ros::ok()){

		// std::cout<<count<<"\n";

		// targetPosition=calc_TargetPosition();
		// targetPosition=obj2_To_Robot;
		// targetOrientation=object_orientation;

		//std::cout << "target position: " << targetPosition[0] << " " << targetPosition[1] << " " << targetPosition[2] << "\n";
		//std::cout << "euc norm: " << targetPosition.norm() << "\n";

		// if(validity_flag){
		if(true){
			_msgTargetPose.position.x=targetPosition[0];
			_msgTargetPose.position.y=targetPosition[1];
			_msgTargetPose.position.z=targetPosition[2];
			_msgTargetPose.orientation.w=targetOrientation[0];
			_msgTargetPose.orientation.x=targetOrientation[1];
			_msgTargetPose.orientation.y=targetOrientation[2];
			_msgTargetPose.orientation.z=targetOrientation[3];

			_pubtarget.publish(_msgTargetPose);
		}

		

		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}



	return 0;

}