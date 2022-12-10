#include "papras_hw/hardware_interface.h"
#include "std_msgs/Float32.h"

void timerCallback(HardwareInterface &hardware_interface,
                   controller_manager::ControllerManager &cm,
                   ros::Time &last_time, ros::Publisher &controlT_pub)
{
  ros::Time curr_time = ros::Time::now();
  ros::Duration elapsed_time = curr_time - last_time;
  last_time = curr_time;

  hardware_interface.read();
  cm.update(ros::Time::now(), elapsed_time);
  hardware_interface.write();
}

int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "papras_control_node");
  ros::NodeHandle node_handle("");
  ros::NodeHandle priv_node_handle("~");
  
  ros::Publisher controlT_pub = node_handle.advertise<std_msgs::Float32>("controlT", 1000);

  HardwareInterface hardware_interface(node_handle, priv_node_handle);
  controller_manager::ControllerManager cm(&hardware_interface, node_handle);

  // update
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(2, &queue);
  spinner.start();
  ros::Time last_time = ros::Time::now();
  ros::TimerOptions timer_options(
    ros::Duration(0.008), // 8ms
    boost::bind(timerCallback, boost::ref(hardware_interface), 
                               boost::ref(cm), 
                               boost::ref(last_time), 
                               boost::ref(controlT_pub)),
    &queue);
  ros::Timer timer = node_handle.createTimer(timer_options);
  ros::spin(); // loop rate is ___hz
  return 0;
}
