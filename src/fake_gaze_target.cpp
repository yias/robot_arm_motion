/** 
 *  Copyright (c) 2020 Iason Batzianoulis
 *  
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  License:    GNU GPL v3
 * 
**/

#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <iostream>
#include <string>
#include <vector>

#include <mutex>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "DSObstacleAvoidance.h"

#include <robot_arm_motion/obstacle_msg.h>

int kb_choice = -1;
bool isRunning;
std::mutex threadMutex;

std::vector<Obstacle> obstacles;
bool _firstObstacleReceived = false;

int kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return ch;
}

void checkKeyBoard()
{

    int key;

    while (isRunning) {
        // std::cin >> key;
        // std::cout << "key: " << key << std::endl;
        if (kbhit()) {
            key = getch();
            if ((key < 48) || (key > 57)) {
                std::cout << "\nNot available choice!! Type a integer from 0 to 9" << std::endl;
            }
            else {
                kb_choice = key - 48;
            }
            std::cout << std::endl;
        }
    }
}

void obstacleListener(const robot_arm_motion::obstacle_msg& msg)
{

    obstacles = std::vector<Obstacle>(msg.obstacles.size());

    for (int i = 0; i < msg.obstacles.size(); i++) {
        // set the obstacle parameters
        obstacles[i]._x0 << msg.obstacles[i].pose.position.x, msg.obstacles[i].pose.position.y, msg.obstacles[i].pose.position.z; // obstacle position
        obstacles[i]._a << msg.obstacles[i].alpha[0], msg.obstacles[i].alpha[1], msg.obstacles[i].alpha[2]; // obstacle axis lengths
        obstacles[i]._p << msg.obstacles[i].power_terms[0], msg.obstacles[i].power_terms[1], msg.obstacles[i].power_terms[2]; // obstacle power terms
        obstacles[i]._safetyFactor = 1.1; // safety factor
        obstacles[i]._rho = 1.1; // reactivity
        obstacles[i]._tailEffect = false;
        obstacles[i]._bContour = true;
    }

    if (!_firstObstacleReceived) {
        _firstObstacleReceived = true;
        ROS_INFO("[robot_arm_motion:doa] Obstacles' poses received\n");
    }
}

int main(int argc, char** argv)
{

    std::thread kbThread; // the thread for getting input from keyboard

    ros::init(argc, argv, "fake_gaze_target");

    ros::NodeHandle _n;

    ros::Publisher fake_target_pub = _n.advertise<std_msgs::Int8>("/fake_target", 1);

    ros::Subscriber obj_sub = _n.subscribe("/obstacles", 1, obstacleListener);

    std_msgs::Int8 pub_msg;

    ros::Rate loop_rate(100);

    isRunning = true;
    kbThread = std::thread(&checkKeyBoard);

    std::cout << "Type an integer number that corresponds to a visible obstacle" << std::endl;

    while (ros::ok()) {

        pub_msg.data = (int8_t)kb_choice;

        fake_target_pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    isRunning = false;
    if (kbThread.joinable()) {
        kbThread.join();
    }

    return 0;
}