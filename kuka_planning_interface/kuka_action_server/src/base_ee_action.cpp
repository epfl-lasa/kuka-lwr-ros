#include "kuka_action_server/base_ee_action.h"

namespace asrv{

Base_ee_action::Base_ee_action(ros::NodeHandle&   nh){

    position_pub  = nh.advertise<geometry_msgs::Pose>("/lwr/one_task_inverse_kinematics/command_pos", 1);
    velocity_pub  = nh.advertise<geometry_msgs::Twist>("/lwr/one_task_inverse_kinematics/command_vel", 1);
    stiffness_pub = nh.advertise<std_msgs::Float64MultiArray>("/lwr/one_task_inverse_kinematics/stiffness",1);
    damping_pub   = nh.advertise<std_msgs::Float64MultiArray>("/lwr/one_task_inverse_kinematics/damping",1);

    ee_damp_msg.data.resize(KUKA_NUM_JOINTS);
    ee_stiff_msg.data.resize(KUKA_NUM_JOINTS);

    pose_sub     = nh.subscribe("/lwr/ee_pose",1,&Base_ee_action::pose_callback,this);
    b_received   = false;

}

void Base_ee_action::sendPose(const geometry_msgs::Pose & pose_msg) {
     position_pub.publish(pose_msg);
}

void Base_ee_action::sendVel(const geometry_msgs::Twist& twist_){
    velocity_pub.publish(twist_);
}

void Base_ee_action::sendStiff(const std_msgs::Float64MultiArray& stiff_msg){
    stiffness_pub.publish(stiff_msg);
}

void Base_ee_action::sendDamp(const std_msgs::Float64MultiArray& damp_msg){
    damping_pub.publish(damp_msg);
}

void Base_ee_action::pose_callback(const geometry_msgs::PoseConstPtr &msg){

    ee_pose_current.setOrigin(tf::Vector3(msg->position.x,msg->position.y,msg->position.z));
    ee_pose_current.setRotation(tf::Quaternion(msg->orientation.x,
                                               msg->orientation.y,
                                               msg->orientation.z,
                                               msg->orientation.w));
    b_received = true;
}

}
