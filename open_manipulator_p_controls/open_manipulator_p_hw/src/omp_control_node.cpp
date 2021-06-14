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

void timerCallback(open_manipulator_p_hw::HardwareInterface &hardware_interface,
                   controller_manager::ControllerManager &cm,
                   ros::Time &last_time)
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
  ros::init(argc, argv, "open_manipulator_p_hw");
  ros::NodeHandle node_handle("");
  ros::NodeHandle priv_node_handle("~");
  open_manipulator_p_hw::HardwareInterface hardware_interface(node_handle, priv_node_handle);
  controller_manager::ControllerManager cm(&hardware_interface, node_handle);

  // update
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  ros::Time last_time = ros::Time::now();
  ros::TimerOptions timer_options(
    ros::Duration(0.010), // 10ms
    boost::bind(timerCallback, boost::ref(hardware_interface), boost::ref(cm), boost::ref(last_time)),
    &queue);
  ros::Timer timer = node_handle.createTimer(timer_options);
  ros::spin();
  return 0;
}
