

/***
 *      a node for listening to optitrack markers (rigid bodies)
 *      and define the position and orienation of the target 
 *      with respect to the robot
 *      
 *      source file of the markerTarget class
 * 
 * 
 */


 #include "markerTarget.hpp"


markerTarget* markerTarget::me = NULL;

markerTarget::markerTarget(ros::NodeHandle &n, double frequency):_n(n), _loopRate(frequency){
    
    ROS_INFO_STREAM("The markerTarget node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");

    // deifne target offset
    _targetOffsets.setConstant(0.0);
 }


bool markerTarget::init(){

    _targetPosition.setConstant(0.0);
    _targetOrientation.setConstant(0.0);
    _targetRotMatrix.setConstant(0.0);
    _mkrPosition.setConstant(0.0);
    _mkrOrientation.setConstant(0.0);
    _mkrRotMat.setConstant(0.0);
    _rbtPosition.setConstant(0.0);
    _rbtOrientation.setConstant(0.0);
    _rbtRotMat.setConstant(0.0);
    _vcurr.setConstant(0.0);
    _stop = false;

	_rotMatrix << 1.0, 0.0, 0.0,
				   0.0, 1.0, 0.0,
				   0.0, 0.0, 1.0;
	
	alpha = 1;

    // Catch CTRL+C event with the callback provided
	signal(SIGINT, markerTarget::stopNodeCallback);

    _targetOffsets << -0.3, 0.0, 0.30;

	

	_f = boost::bind(&markerTarget::dynRecCallback, this, _1, _2);

	dynServer.setCallback(_f);

    // Subscriber definitions
    _mkrSub = _n.subscribe("/vrpn_client_node/hand/pose", 1, &markerTarget::updateMkrPose, this, ros::TransportHints().reliable().tcpNoDelay());
    _robotBaseSub = _n.subscribe("/vrpn_client_node/robot_right/pose", 1, &markerTarget::updateRobotBasePose, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
    _pubtarget = _n.advertise<geometry_msgs::Pose>("/target", 1);

    // Check if node OK
	if (_n.ok()) 
	{ 
		// Wait for poses being published
        while(!(_firstMarkerPoseReceived && _firstRbtBasePoseReceived)){
            ros::spinOnce();
        }
		ROS_INFO("The motion generator is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}

int markerTarget::run(){

    _initTime = ros::Time::now().toSec();

    while (!_stop) 
	{
		// Check if we received the robot pose and foot data
		// if(_firstRealPoseReceived)
		// {
		// Compute control command
	    computeDesiredTarget();

			// // Start monitoring the keyboard
			// if (getch() == ' ')
			// {
			// 	_errorButtonPressed = true;
			// 	_errorButtonCounter = 0;
			// 	ROS_INFO_STREAM("Received key press");
			// }

			// // Log data
			// logData();

		// Publish data to topics
		publishData();
		
        // }
		// else
		// {
		// 	_initTime = ros::Time::now().toSec();
		// }

		ros::spinOnce();

		_loopRate.sleep();
	}

	
    // Close ros
	ros::shutdown();

    return 0;

}

// function for monitoring ctrl+c
void markerTarget::stopNodeCallback(int sig)
{
	me->_stop = true;
}


// callback functions

void markerTarget::updateRobotBasePose(const geometry_msgs::PoseStamped& msg){
    /*
    *   Callback function for updating the robot's base pose
    * 
    */
   	
    // extract position from the message
	_rbtPosition << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    // extract orientation from the message
  	_rbtOrientation << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;
	_rbtRotMat = Utils<float>::quaternionToRotationMatrix(_rbtOrientation);


	if(!_firstRbtBasePoseReceived)
	{
		ROS_INFO("Initial robot base pose received\n");
		_firstRbtBasePoseReceived = true;
		_rbtTimeStamp=msg.header.seq;
		
	}
}


void markerTarget::updateMkrPose(const geometry_msgs::PoseStamped& mocapmsg){

    /*
    *   Callback function for updating the pose of the marker --
    * 
    */
    

    // extract position from the message
    _mkrPosition << mocapmsg.pose.position.x, mocapmsg.pose.position.y, mocapmsg.pose.position.z;
	// object_position[0]=mocapmsg.pose.position.x;
	// object_position[1]=mocapmsg.pose.position.y;
	// object_position[2]=mocapmsg.pose.position.z+_objectZOffset;

	// extract orientation from the message
	_mkrOrientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

	_mkrRotMat = Utils<float>::quaternionToRotationMatrix(_mkrOrientation);

	
	if(!_firstMarkerPoseReceived)
	{
		ROS_INFO("Initial marker pose received\n");

		// update the stamp of the mocap system
		_mkrTimeStamp=mocapmsg.header.seq;

        _firstMarkerPoseReceived=true;
		
	}

}


void markerTarget::publishData()
{
    /*
    *   Function for publishing the desired target pose
    * 
    */

	_mutex.lock();

    _tagetPoseMsg.position.x=-_targetPosition[1];
	_tagetPoseMsg.position.y=_targetPosition[0];
	_tagetPoseMsg.position.z=_targetPosition[2];
	_tagetPoseMsg.orientation.w=_targetOrientation[0];
	_tagetPoseMsg.orientation.x=_targetOrientation[1];
	_tagetPoseMsg.orientation.y=_targetOrientation[2];
	_tagetPoseMsg.orientation.z=_targetOrientation[3];

	
	_pubtarget.publish(_tagetPoseMsg);

	_mutex.unlock();
}


int markerTarget::computeDesiredTarget(){

    /*
    *   Function for publishing the desired target pose
    * 
    */
   
    // _targetPosition = _mkrPosition - _rbtPosition + _targetOffsets;

    _targetOrientation[0] = 0.0;
    _targetOrientation[1] = 0.0;
    _targetOrientation[2] = 1.0;
    _targetOrientation[3] = 0.0;

	_targetPosition = alpha * _rotMatrix * (_mkrPosition - _rbtPosition) + _targetOffsets;

    std::cout << "target position: x: " << -_targetPosition[1] << ", y: " << _targetPosition[0] << ", z: " << _targetPosition[2] << std::endl; 



   return 0;
}


void markerTarget::dynRecCallback(robot_arm_motion::targetOffsetsConfig &config, uint32_t level){
	_targetOffsets << config.x_offset, config.y_offset, config.z_offset;
}