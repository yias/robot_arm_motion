#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>

#include "Utils.h"
#include "gaze_tracking/coordinate.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h> 



// ----------------- variables for the robot base -----------------

bool _firstKukaBasePoseReceived=false;
double robotBaseStamp=0;

Eigen::VectorXf robot_base_position(3); 	// the position of the robot's base as received from the mocap system
Eigen::VectorXf robot_base_orienation(4);	// the orienation of the robot's base
Eigen::Matrix3f robot_base_rotMatrix;		// the rotation matrix of the orientation of the robot's base 



// ----------------- variables for the object 1 -----------------

bool _firstObjectPoseReceived=false;
double objectPoseStamp=0;
float _objectZOffset=0.1f;
float _objectYOffset=0.0f;

Eigen::VectorXf object_position(3);									// the position of the object as received from the mocap system
Eigen::VectorXf object_orientation(4);								// the orientation of the object as received from the mocap system
Eigen::Matrix3f object_rotMatrix;


// ----------------- variables for the object 2 -----------------

bool _firstObject2PoseReceived=false;
double object2PoseStamp=0;
float _object2ZOffset=0.1f;
float _object2YOffset=0.0f;

Eigen::VectorXf object2_position(3);								// the position of the object 2 as received from the mocap system
Eigen::VectorXf object2_orientation(4);								// the orientation of the object 2 as received from the mocap system
Eigen::Matrix3f object2_rotMatrix;


// ----------------- variables for the ARCU-board frame -----------------

bool _firstARCUPoseReceived=false;
double ARCUboard_PoseStamp=0;
float _ARCUboard_ZOffset=0.01f;
float _ARCUboard_YOffset=0.0f;

Eigen::VectorXf ARCUboard_position(3);								// the position of the frame on the ARCU-borad as received from the mocap system
Eigen::VectorXf ARCUboard_orientation(4);							// the orientation of the frame on the ARCU-borad as received from the mocap system
Eigen::Matrix3f ARCUboard_rotMatrix;


// ----------------- variables for the target -----------------

bool _TargetPoseReceived=false;
float _Target_XOffset=0.0f;
float _Target_YOffset=0.0f;
float _Target_ZOffset=0.0f;

Eigen::VectorXf ARCU_Target_position(2);							// the position of the target on the ARCU-borad as received from the gaze tracker
Eigen::Vector4f gt_msg_coord;
bool gt_flag, validity_flag=false;
double classif_threshold=0.1;




// ----------------- callback function for the base listener -----------------

void robotBaseListener(const geometry_msgs::PoseStamped& msg){

	robot_base_position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  	robot_base_orienation << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
	robot_base_rotMatrix = Utils::quaternionToRotationMatrix(robot_base_orienation);


	if(!_firstKukaBasePoseReceived)
	{
		ROS_INFO("Initial robot base pose received\n");
		_firstKukaBasePoseReceived = true;
		robotBaseStamp=msg.header.seq;
		
		
	}
}

// ----------------- callback function for object 1 -----------------

void object1Listener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for acquiring the position of the base of the robot arm --*/
    

    // extract position from the message
	object_position[0]=mocapmsg.pose.position.x;
	object_position[1]=mocapmsg.pose.position.y;
	object_position[2]=mocapmsg.pose.position.z+_objectZOffset;

	// extract orientation from the message
	object_orientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

	object_rotMatrix=Utils::quaternionToRotationMatrix(object_orientation);

	
	if(!_firstObjectPoseReceived)
	{
			
		_firstObjectPoseReceived=true;
		ROS_INFO("Initial object 1 pose received\n");

		// update the stamp of the mocap system
		objectPoseStamp=mocapmsg.header.seq;
		
	}


    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}


// ----------------- callback function for object 2 -----------------

void object2Listener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for acquiring the position of the base of the robot arm --*/
    

    // extract position from the message
	object2_position[0]=mocapmsg.pose.position.x;
	object2_position[1]=mocapmsg.pose.position.y;
	object2_position[2]=mocapmsg.pose.position.z+_objectZOffset;

	// extract orientation from the message
	object2_orientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

	object2_rotMatrix=Utils::quaternionToRotationMatrix(object2_orientation);

	
	if(!_firstObject2PoseReceived)
	{
			
		_firstObject2PoseReceived=true;
		ROS_INFO("Initial object 2 pose received\n");

		// update the stamp of the mocap system
		object2PoseStamp=mocapmsg.header.seq;
		
	}


    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}


// ----------------- callback function for the ARCU-board frame -----------------

void ARCUboard_Listener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for acquiring the position of the base of the robot arm --*/
    

    // extract position from the message
	ARCUboard_position[0]=mocapmsg.pose.position.x;
	ARCUboard_position[1]=mocapmsg.pose.position.y;
	ARCUboard_position[2]=mocapmsg.pose.position.z+_objectZOffset;

	// extract orientation from the message
	ARCUboard_orientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

	ARCUboard_rotMatrix=Utils::quaternionToRotationMatrix(ARCUboard_orientation);

	
	if(!_firstARCUPoseReceived)
	{
			
		_firstARCUPoseReceived=true;
		ROS_INFO("Initial ARCU-board pose received\n");

		// update the stamp of the mocap system
		object2PoseStamp=mocapmsg.header.seq;
		
	}


    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}

// ----------------- callback function for the ARCU-board frame -----------------

void gazeTracking_Listener(const gaze_tracking::coordinate msg){

	std::cout<<"okokook\n";
	gt_flag=msg.flag;

	if(gt_flag){
		gt_msg_coord << msg.coord[0],  msg.coord[1], msg.coord[2], msg.coord[3];

		ARCU_Target_position << gt_msg_coord[2], gt_msg_coord[3];

		std::cout << "gaze msg" << ARCU_Target_position[0] << " " << ARCU_Target_position[1] << "\n";

		if(!_TargetPoseReceived)
		{			
			_TargetPoseReceived=true;
			ROS_INFO("Initial target position received\n");	
		}
	}
	

	

}

// ----------------- function for calculating the position of the target with respect to the robot frame -----------------

Eigen::Vector3f calc_TargetPosition(){

	Eigen::Vector3f tmp_TPosition;
	Eigen::Vector3f ARCU_To_Robot;
	Eigen::Vector3f obj1_To_Robot;
	Eigen::Vector3f obj2_To_Robot;
	double euc_d1,euc_d2;
	int decision=0;


	// the position of the ARCU-board on the robot frame

	ARCU_To_Robot=ARCUboard_position-robot_base_position;

	// the position of object1 on the robot frame

	obj1_To_Robot=object_position-robot_base_position;

	std::cout << "obj1 pos " << obj1_To_Robot[0] << " " << obj1_To_Robot[1] << " " << obj1_To_Robot[2] << "\n";

	// the position of object2 on the robot frame

	obj2_To_Robot=object2_position-robot_base_position;

	// the position of the object on the robot frame in 2D

	tmp_TPosition[0]=ARCU_To_Robot[0]+ARCU_Target_position[0];
	tmp_TPosition[1]=ARCU_To_Robot[1]+ARCU_Target_position[1];

	// ------ classify the position of the target to one of the two objects with respect to the distance from the objects  --------------------

	// the euclidean distance from object 1

	euc_d1=std::sqrt(std::fabs(tmp_TPosition[0]-obj1_To_Robot[0])+std::fabs(tmp_TPosition[1]-obj1_To_Robot[1]));

	euc_d2=std::sqrt(std::fabs(tmp_TPosition[0]-obj2_To_Robot[0])+std::fabs(tmp_TPosition[1]-obj2_To_Robot[1]));

	std::cout<<"euclidean distances " << euc_d1 << " " << euc_d2 << "\n";

	// classification

	if(euc_d1<=classif_threshold){
		validity_flag=true;
		return obj1_To_Robot;
	}else{
		if(euc_d2<=classif_threshold){
			validity_flag=true;
			return obj2_To_Robot;
		}else{
			// std::cout<<"Invalid target selection\n";
			if(euc_d1<=euc_d2){
				validity_flag=false;
				return obj1_To_Robot;
			}else{
					validity_flag=false;
					return obj2_To_Robot;
				}
			}
		}
	

	// return tmp_TPosition;
}


int main(int argc, char **argv){

	// initialize the node
    ros::init(argc, argv, "handtracker");

	ros::NodeHandle n;

	ros::Subscriber robotBaseSub=n.subscribe("Robot_base/pose", 10, robotBaseListener);

	ros::Subscriber robotObject1Sub=n.subscribe("object1/pose", 10, object1Listener);

	ros::Subscriber robotObject2Sub=n.subscribe("object2/pose", 10, object2Listener);

	ros::Subscriber ARCU_Sub=n.subscribe("ARCU_frame/pose", 10, ARCUboard_Listener);

	ros::Subscriber gazeTracking_Sub=n.subscribe("/coordinate", 10, gazeTracking_Listener);

	ros::Publisher _pubtarget=n.advertise<geometry_msgs::Pose>("target", 10);

	geometry_msgs::Pose _msgTargetPose;

	Eigen::VectorXf targetPosition(3);

	Eigen::VectorXf targetOrientation(4);

	ros::Rate loop_rate(1000);

	while(!_firstKukaBasePoseReceived){
		while(!_firstObjectPoseReceived){
			// std::cout<<"okook\n";
			ros::spinOnce();
			loop_rate.sleep();	
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


	int count = 0;

	while (ros::ok()){

		// std::cout<<count<<"\n";

		targetPosition=calc_TargetPosition();
		targetOrientation=object_orientation;

		//std::cout << "target position: " << targetPosition[0] << " " << targetPosition[1] << " " << targetPosition[2] << "\n";
		//std::cout << "euc norm: " << targetPosition.norm() << "\n";

		if(validity_flag){
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