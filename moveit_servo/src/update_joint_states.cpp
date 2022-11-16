/* Author: Sankalp Yamsani and Kazuki Shin*/
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/JointJog.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>

// Some constants used in the Servo Teleop demo
const std::string JOINT_STATE_TOPIC = "/joint_states";
const std::string ARM1_CONTROLLER_TOPIC = "/arm1_controller/command";
const std::string ARM2_CONTROLLER_TOPIC = "/arm2_controller/command";
const std::string MANUAL_JOINT_STATE_TOPIC = "/servo_joint_states";


ros::Publisher new_joint_state_pub_;
sensor_msgs::JointState current_joint_state; 
std::unordered_map<std::string, int> indexes_arm1;
std::unordered_map<std::string, int> indexes_arm2;
std::unordered_map<std::string, double> last_joint_values;


void joint_state_callback(const sensor_msgs::JointState msg) {
    current_joint_state = msg;

}

void arm1_command_callback(const trajectory_msgs::JointTrajectory msg) {
    sensor_msgs::JointState publish_joint_state = current_joint_state;
    for(size_t i = 0; i < msg.joint_names.size(); i++){
        indexes_arm1[msg.joint_names[i]] = i;
    }

    for(size_t i = 0; i < publish_joint_state.name.size(); i++){
        if(indexes_arm1.find(publish_joint_state.name[i]) != indexes_arm1.end()){
            publish_joint_state.position[i] = msg.points[msg.points.size()-1].positions[indexes_arm1[publish_joint_state.name[i]]];
            last_joint_values[publish_joint_state.name[i]] = publish_joint_state.position[i];
        }else if(last_joint_values.find(publish_joint_state.name[i]) != last_joint_values.end()){
            publish_joint_state.position[i] = last_joint_values[publish_joint_state.name[i]];
            last_joint_values[publish_joint_state.name[i]] = publish_joint_state.position[i];
        }
    }
    new_joint_state_pub_.publish(publish_joint_state);

}


void arm2_command_callback(const trajectory_msgs::JointTrajectory msg){
    sensor_msgs::JointState publish_joint_state = current_joint_state;
    for(size_t i = 0; i < msg.joint_names.size(); i++){
        indexes_arm2[msg.joint_names[i]] = i;
    }

    for(size_t i = 0; i < publish_joint_state.name.size(); i++){
        if(indexes_arm2.find(publish_joint_state.name[i]) != indexes_arm2.end()){
            publish_joint_state.position[i] = msg.points[msg.points.size()-1].positions[indexes_arm2[publish_joint_state.name[i]]];
            last_joint_values[publish_joint_state.name[i]] = publish_joint_state.position[i];
        }else if(last_joint_values.find(publish_joint_state.name[i]) != last_joint_values.end()){
            publish_joint_state.position[i] = last_joint_values[publish_joint_state.name[i]];
            last_joint_values[publish_joint_state.name[i]] = publish_joint_state.position[i];
        }
    }
    new_joint_state_pub_.publish(publish_joint_state);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_joint_states");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    new_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>(MANUAL_JOINT_STATE_TOPIC,1000,true);

    ros::Subscriber joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC,10, joint_state_callback);
    ros::Subscriber arm1_command = nh.subscribe(ARM1_CONTROLLER_TOPIC,10,arm1_command_callback);
    ros::Subscriber arm2_command = nh.subscribe(ARM2_CONTROLLER_TOPIC,10,arm2_command_callback);

    
    sensor_msgs::JointState publish_joint_state = current_joint_state;

    while(indexes_arm1.size() == 0 && indexes_arm2.size() == 0){
        publish_joint_state = current_joint_state;
        new_joint_state_pub_.publish(publish_joint_state);
        ros::Duration(0.1).sleep();

    }
    
    ros::waitForShutdown();
}