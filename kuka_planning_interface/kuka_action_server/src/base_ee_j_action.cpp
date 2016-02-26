#include "kuka_action_server/base_ee_j_action.h"



namespace asrv{

Base_ee_j_action::Base_ee_j_action(ros::NodeHandle&   nh, const std::string &controller_name){

    position_pub  = nh.advertise<geometry_msgs::Pose>("/lwr/"         + controller_name + "/command_pos", 1);
    velocity_pub  = nh.advertise<geometry_msgs::Twist>("/lwr/"        + controller_name + "/command_vel", 1);
    joint_pub     = nh.advertise<std_msgs::Float64MultiArray>("lwr/"  + controller_name + "/command_joint_pos",1);
    stiffness_pub = nh.advertise<std_msgs::Float64MultiArray>("/lwr/" + controller_name + "/stiffness",1);
    damping_pub   = nh.advertise<std_msgs::Float64MultiArray>("/lwr/" + controller_name + "/damping",1);
    orient_pub    = nh.advertise<geometry_msgs::Quaternion>("/lwr/"   + controller_name + "/command_orient",1);
    string_pub    = nh.advertise<std_msgs::String>("/lwr/"            + controller_name + "/command_string",1);
    grav_pub      = nh.advertise<std_msgs::Bool>("/lwr/"              + controller_name + "/command_grav",1);

    damp_msg.data.resize(KUKA_NUM_JOINTS);
    stiff_msg.data.resize(KUKA_NUM_JOINTS);
    joint_pos_msg.data.resize(KUKA_NUM_JOINTS);

    pose_sub     = nh.subscribe("/lwr/ee_pose",1,&Base_ee_j_action::pose_callback,this);
    b_received   = false;
}

void Base_ee_j_action::sendCartPose(const geometry_msgs::Pose & pose_msg) {
     position_pub.publish(pose_msg);
}

void Base_ee_j_action::sendCartVel(const geometry_msgs::Twist& twist_){
    velocity_pub.publish(twist_);
}

void Base_ee_j_action::sendStiff(const std_msgs::Float64MultiArray& stiff_msg){
    stiffness_pub.publish(stiff_msg);
}

void Base_ee_j_action::sendJointPos(const std_msgs::Float64MultiArray &joint_msg){
    joint_pub.publish(joint_msg);
}

void Base_ee_j_action::sendDamp(const std_msgs::Float64MultiArray& damp_msg){
    damping_pub.publish(damp_msg);
}

void Base_ee_j_action::sendGrav(const std_msgs::Bool &grav_msg){
    grav_pub.publish(grav_msg);
}

void Base_ee_j_action::sendOrient(const geometry_msgs::Quaternion& orient_msg){
    orient_pub.publish(orient_msg);
}

void Base_ee_j_action::sendString(const std_msgs::String& string_msg){
    string_pub.publish(string_msg);
}

void Base_ee_j_action::pose_callback(const geometry_msgs::PoseConstPtr &msg){

    ee_pose_current.setOrigin(tf::Vector3(msg->position.x,msg->position.y,msg->position.z));
    ee_pose_current.setRotation(tf::Quaternion(msg->orientation.x,
                                               msg->orientation.y,
                                               msg->orientation.z,
                                               msg->orientation.w));
    b_received = true;
}

}
