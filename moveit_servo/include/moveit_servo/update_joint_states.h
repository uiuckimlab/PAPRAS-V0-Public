/* Author: Sankalp Yamsani and Kazuki Shin*/
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/JointJog.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>

class UpdateJointStates{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joint_state_sub_;
        ros::Subscriber arm_controller_sub_;
        ros::Subscriber pause_servoing_sub_;
        ros::Publisher new_joint_state_pub_; 

        bool estimated_publish_ = true;
        std::unordered_map<std::string, int> indexes_;
        sensor_msgs::JointState current_joint_state; 


        // names 
        std::string predicted_arm_position_topic_;

        std::string joint_state_topic = "/joint_states";
        std::string servo_joint_state_topic_ = "/servo_joint_states";       
         
    public:
        UpdateJointStates(ros::NodeHandle nh, std::string out_topic, std::string pause_servoing_topic);

        void command_callback(const trajectory_msgs::JointTrajectory msg);

        void pause_servoing_callback(const std_msgs::Bool msg);

        void joint_state_callback(const sensor_msgs::JointState msg);
};