/***
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 * 
 *      a node for listening to optitrack markers (rigid bodies)
 *      and define the position and orienation of the target 
 *      with respect to the robot
 *      
 * 
 */

#include "markerTarget.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "markerTarget");
  ros::NodeHandle n;
  float frequency = 100.0f;

  markerTarget mrkrTarget(n,frequency);
 
  if (!mrkrTarget.init()) 
  {
    return -1;
  }
  else
  {
    mrkrTarget.run();
  }

  return 0;
  
}