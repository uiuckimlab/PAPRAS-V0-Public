// ROS
#include <ros/ros.h>

enum state
{
    STATE_INIT,
    STATE_PAUSED,
    STATE_DONE
};

int main(int argc, char** argv)
{

  // Initialize ROS Node
  ros::init(argc, argv, "chess_moveit");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  // State Machine
  while (ros::ok())
  {

  }

}
