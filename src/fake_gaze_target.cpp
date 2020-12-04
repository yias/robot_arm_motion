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

int kb_choice = -1;
bool isRunning;
std::mutex threadMutex;

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

int main(int argc, char** argv)
{

    std::thread kbThread; // the thread for getting input from keyboard

    ros::init(argc, argv, "fake_gaze_target");

    ros::NodeHandle _n;

    ros::Publisher fake_target_pub = _n.advertise<std_msgs::Int8>("/fake_target", 1);

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