/* Author: Sankalp Yamsani and Kazuki Shin*/
#include <moveit_servo/update_joint_states.h>


UpdateJointStates::UpdateJointStates(ros::NodeHandle nh, std::string out_topic, std::string pause_servoing_topic) :
    nh_(nh)
{
    predicted_arm_position_topic_ = out_topic;
    arm_controller_sub_ = nh_.subscribe(out_topic,10, &UpdateJointStates::command_callback, this);
    joint_state_sub_ = nh_.subscribe(joint_state_topic,10, &UpdateJointStates::joint_state_callback, this);
    pause_servoing_sub_ = nh_.subscribe(pause_servoing_topic,10,&UpdateJointStates::pause_servoing_callback, this);
    new_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(servo_joint_state_topic_,1000,true);
}

void UpdateJointStates::command_callback(const trajectory_msgs::JointTrajectory msg){
    if(!estimated_publish_){
        return;
    }
    sensor_msgs::JointState publish_joint_state;
    publish_joint_state.header = current_joint_state.header;
    for(size_t i = 0; i < msg.joint_names.size(); i++){
        indexes_[msg.joint_names[i]] = i;
    }
    for(size_t i = 0; i < current_joint_state.name.size(); i++){
        if(indexes_.find(current_joint_state.name[i]) != indexes_.end()){
            publish_joint_state.name.push_back(current_joint_state.name[i]);
            publish_joint_state.position.push_back(msg.points[msg.points.size()-1].positions[indexes_[current_joint_state.name[i]]]);
            publish_joint_state.velocity.push_back(current_joint_state.velocity[i]);
            publish_joint_state.effort.push_back(current_joint_state.effort[i]);
        }
    }
    new_joint_state_pub_.publish(publish_joint_state);

}

void UpdateJointStates::pause_servoing_callback(const std_msgs::Bool msg){
    estimated_publish_ = !msg.data;
}

void UpdateJointStates::joint_state_callback(const sensor_msgs::JointState msg){
    current_joint_state = msg;
    if(!estimated_publish_ || indexes_.size() == 0){
        new_joint_state_pub_.publish(msg);
    }
}


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "update_joint_states");

//     ros::NodeHandle nh;

//     ros::AsyncSpinner spinner(0);
//     spinner.start();
    
//     UpdateJointStates left_arm(nh, "/arm1_controller/command", "/servo_server/set_servoing_paused");

    
//     ros::waitForShutdown();
// }