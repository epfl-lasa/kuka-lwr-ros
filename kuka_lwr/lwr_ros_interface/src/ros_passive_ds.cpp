#include "lwr_ros_interface/ros_passive_ds.h"

namespace ros_controller_interface{


Ros_passive_ds::Ros_passive_ds(ros::NodeHandle&   nh, const std::string& name_space,const std::string& controller_name){

    velocity_pub        = nh.advertise<geometry_msgs::Twist>(       "/" + name_space + "/" + controller_name + "/passive_ds_command_vel", 1);
    rot_stiffness_pub   = nh.advertise<std_msgs::Float64>(          "/" + name_space + "/" + controller_name + "/passive_ds_stiffness",1);
    rot_damp_pub        = nh.advertise<std_msgs::Float64>(          "/" + name_space + "/" + controller_name + "/passive_ds_damping",1);
    orient_pub          = nh.advertise<geometry_msgs::Quaternion>(  "/" + name_space + "/" + controller_name + "/passive_ds_command_orient",1);
    passive_ds_eig_pub  = nh.advertise<std_msgs::Float64MultiArray>("/" + name_space + "/" + controller_name + "/passive_ds_eig",1);

    eig_msg.data.resize(2);

}


void Ros_passive_ds::sendDampEig(const std_msgs::Float64MultiArray& msg){
    passive_ds_eig_pub.publish(msg);
}

void Ros_passive_ds::sendCartVel(const geometry_msgs::Twist& twist_){
    velocity_pub.publish(twist_);
}

void Ros_passive_ds::sendRotStiff(const std_msgs::Float64& stiff_msg){
    rot_stiffness_pub.publish(stiff_msg);
}

void Ros_passive_ds::sendRotDamp(const std_msgs::Float64& damp_msg){
    rot_damp_pub.publish(damp_msg);
}

void Ros_passive_ds::sendOrient(const geometry_msgs::Quaternion &orient_msg){
    orient_pub.publish(orient_msg);
}

}
