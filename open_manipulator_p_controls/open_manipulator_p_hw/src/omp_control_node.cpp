/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#include "open_manipulator_p_hw/hardware_interface.h"
#include "std_msgs/Float32.h"

void timerCallback(open_manipulator_p_hw::HardwareInterface &hardware_interface,
                   controller_manager::ControllerManager &cm,
                   ros::Time &last_time, ros::Publisher &controlT_pub,
                   ros::Publisher &readT_pub, ros::Publisher &updateT_pub, ros::Publisher &writeT_pub)
{
  ros::Time curr_time = ros::Time::now();
  ros::Duration elapsed_time = curr_time - last_time;
  last_time = curr_time;

  hardware_interface.read();
  ros::Time read_time = ros::Time::now();
  cm.update(ros::Time::now(), elapsed_time);
  ros::Time update_time = ros::Time::now();
  hardware_interface.write();
  ros::Time write_time = ros::Time::now();

  std_msgs::Float32 controlT_msg;
  controlT_msg.data = (write_time - curr_time).toSec();
  controlT_pub.publish(controlT_msg);

  std_msgs::Float32 readT_msg;
  readT_msg.data = (read_time - curr_time).toSec();
  readT_pub.publish(readT_msg);

  std_msgs::Float32 updateT_msg;
  updateT_msg.data = (update_time - read_time).toSec();
  updateT_pub.publish(updateT_msg);

  std_msgs::Float32 writeT_msg;
  writeT_msg.data = (write_time - update_time).toSec();
  writeT_pub.publish(writeT_msg);
}

int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "open_manipulator_p_hw");
  ros::NodeHandle node_handle("");
  ros::NodeHandle priv_node_handle("~");
  
  ros::Publisher controlT_pub = node_handle.advertise<std_msgs::Float32>("controlT", 1000);
  ros::Publisher readT_pub = node_handle.advertise<std_msgs::Float32>("readT", 1000);
  ros::Publisher updateT_pub = node_handle.advertise<std_msgs::Float32>("updateT", 1000);
  ros::Publisher writeT_pub = node_handle.advertise<std_msgs::Float32>("writeT", 1000);

  open_manipulator_p_hw::HardwareInterface hardware_interface(node_handle, priv_node_handle);
  controller_manager::ControllerManager cm(&hardware_interface, node_handle);

  // update
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  ros::Time last_time = ros::Time::now();
  ros::TimerOptions timer_options(
    ros::Duration(0.004), // 10ms
    boost::bind(timerCallback, boost::ref(hardware_interface), 
                               boost::ref(cm), 
                               boost::ref(last_time), 
                               boost::ref(controlT_pub),
                               boost::ref(readT_pub),
                               boost::ref(updateT_pub),
                               boost::ref(writeT_pub)),
    &queue);
  ros::Timer timer = node_handle.createTimer(timer_options);
  ros::spin(); // loop rate is 250hz
  return 0;
}
