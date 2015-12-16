#include "kuka_action_server/base_j_action.h"
#include "sensor_msgs/JointState.h"
#include <assert.h>
#include <boost/lexical_cast.hpp>

namespace asrv{

Base_j_action::Base_j_action(ros::NodeHandle &nh)
{

    joint_sensor_sub     = nh.subscribe("/lwr/joint_states",1,&Base_j_action::jStateCallback,this);
    joint_cmd_pub        = nh.advertise<std_msgs::Float64MultiArray>("/lwr/joint_position_impedance_controller/command",1);
    joint_stiff_pub      = nh.advertise<std_msgs::Float64MultiArray>("/lwr/joint_position_impedance_controller/stiffness",1);
    joint_damp_pub       = nh.advertise<std_msgs::Float64MultiArray>("/lwr/joint_position_impedance_controller/damping",1);



    joint_cmd_msg.data.resize(KUKA_NUM_JOINTS);
    joint_sensed.resize(KUKA_NUM_JOINTS);
    joint_sensed.setZero();

    // both stiffness and damping values are sent in this vector
    joint_stiff_msg.data.resize(KUKA_NUM_JOINTS);
    joint_damp_msg.data.resize(KUKA_NUM_JOINTS);
}

void Base_j_action::update_position(const Eigen::VectorXd &joint_position){

    if(joint_position.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("Base_j_action::update joint_position.n_elem != 7");
        return;
    }

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_cmd_msg.data[i] = joint_position(i);
    }

    joint_cmd_pub.publish(joint_cmd_msg);
}



void Base_j_action::update_stiffness(const Eigen::VectorXd& joint_stiffness){

    if(joint_stiffness.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("Base_j_action::update joint_stiffness.n_elem != 7");
        return;
    }

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_stiff_msg.data[i] = joint_stiffness(i);
    }

    joint_stiff_pub.publish(joint_stiff_msg);

}

void Base_j_action::update_damping(const Eigen::VectorXd& joint_damping){

    if(joint_damping.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("Base_j_action::update joint_damping.n_elem != 7");
        return;
    }

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_damp_msg.data[i] = joint_damping(i);
    }

    joint_damp_pub.publish(joint_damp_msg);

}



void Base_j_action::jStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_sensed(i)      = msg->position[i];
    }
}


}


