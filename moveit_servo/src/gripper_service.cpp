/* Author: Sankalp Yamsani and Kazuki Shin*/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

const std::string GRIPPER_RIGHT_TOPIC = "right_arm_servo_server/gripper_right";
const std::string GRIPPER_LEFT_TOPIC = "left_arm_servo_server/gripper_left";
const float GOAL_SECS = 0.5;

class GripperMove{
    private:
        ros::NodeHandle nh;
        ros::Subscriber gripper_sub;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_trajectory_action_;
        std::vector<std::string> gripper_name;
        bool using_soft_gripper_;
    public:
        GripperMove(ros::NodeHandle nh_, std::string gripper_action_name, std::string gripper_subscriber_topic,  std::vector<std::string> gripper_name_,bool using_soft_gripper = false) :
            nh(nh_),
            gripper_trajectory_action_(gripper_action_name, true)
        {

            gripper_sub = nh.subscribe(gripper_subscriber_topic,10, &GripperMove::gripper_cb, this);
            gripper_name = gripper_name_;
            using_soft_gripper_ = using_soft_gripper;
            while(!gripper_trajectory_action_.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }
            ROS_INFO("Gripper Found");
        }

        void gripper_cb(const std_msgs::Float32 msg){
            control_msgs::FollowJointTrajectoryGoal goal;
            for(size_t i = 0; i < gripper_name.size(); i++){
                goal.trajectory.joint_names.push_back(gripper_name[i]);
            }
            trajectory_msgs::JointTrajectoryPoint joint_angle;
            if(!using_soft_gripper_){
                joint_angle.positions.push_back(msg.data);
                joint_angle.time_from_start = ros::Duration(GOAL_SECS);
            }else{
                joint_angle.positions.push_back((msg.data * (1.78-0.1)) + 0.1); // 1.78 max 0.1 min
                joint_angle.positions.push_back(0); // for pulley
                joint_angle.time_from_start = ros::Duration(GOAL_SECS);
            }
            
            goal.trajectory.points.push_back(joint_angle);
            goal.trajectory.header.stamp = ros::Time::now();
            gripper_trajectory_action_.sendGoal(goal);
        }

};


int main(int argc, char **argv){
    ros::init(argc, argv, "gripper_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    GripperMove left_gripper(nh,"/soft_gripper1_controller/follow_joint_trajectory", GRIPPER_LEFT_TOPIC, {"robot1/soft_gripper_jaw","robot1/soft_gripper_pulley"},true);
    GripperMove right_gripper(nh,"/soft_gripper2_controller/follow_joint_trajectory", GRIPPER_RIGHT_TOPIC, {"robot2/soft_gripper_jaw","robot2/soft_gripper_pulley",},true);

    ros::waitForShutdown();

    return 0;

}