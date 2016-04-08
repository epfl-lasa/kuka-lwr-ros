#include "lwr_ros_interface/ros_joint.h"
#include "sensor_msgs/JointState.h"
#include <assert.h>
#include <boost/lexical_cast.hpp>

namespace ros_controller_interface{

Ros_joint::Ros_joint(ros::NodeHandle &nh, const std::string &controller_name)
{

    joint_sensor_sub     = nh.subscribe("/lwr/joint_states",10,&Ros_joint::jStateCallback,this);
    joint_cmd_pub        = nh.advertise<std_msgs::Float64MultiArray>("/lwr/" + controller_name + "/command_joint_pos",1);
    joint_stiff_pub      = nh.advertise<std_msgs::Float64MultiArray>("/lwr/" + controller_name + "/stiffness",1);
    joint_damp_pub       = nh.advertise<std_msgs::Float64MultiArray>("/lwr/" + controller_name + "/damping",1);


    //joint_state_msg.position.resize(KUKA_NUM_JOINTS);

    joint_cmd_msg.data.resize(KUKA_NUM_JOINTS);
    joint_sensed.resize(KUKA_NUM_JOINTS);
    joint_sensed.setZero();


    // both stiffness and damping values are sent in this vector
    joint_stiff_msg.data.resize(KUKA_NUM_JOINTS);
    joint_damp_msg.data.resize(KUKA_NUM_JOINTS);
}

void Ros_joint::sendJointPos(const Eigen::VectorXd &joint_position){

    if(joint_position.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("Base_j_action::update joint_position.n_elem != 7");
        return;
    }

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_cmd_msg.data[i] = joint_position(i);
    }

    joint_cmd_pub.publish(joint_cmd_msg);
}



void Ros_joint::sendStiff(const std_msgs::Float64MultiArray& stiff_msg){
    joint_stiff_pub.publish(stiff_msg);
}

void Ros_joint::sendDamp(const std_msgs::Float64MultiArray &damp_msg){
    joint_damp_pub.publish(damp_msg);
}

void Ros_joint::jStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
       joint_sensed[i]     = msg->position[i];
    }
}


}


