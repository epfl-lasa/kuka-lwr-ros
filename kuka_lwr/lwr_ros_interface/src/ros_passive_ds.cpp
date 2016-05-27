#include "lwr_ros_interface/ros_passive_ds.h"

namespace ros_controller_interface{


Ros_passive_ds::Ros_passive_ds(ros::NodeHandle&   nh, const std::string& name_space,const std::string& controller_name){

    velocity_pub  = nh.advertise<geometry_msgs::Twist>("/"          + name_space + "/" + controller_name + "/passive_ds_command_vel", 1);
    stiffness_pub = nh.advertise<std_msgs::Float64MultiArray>("/"   + name_space + "/" + controller_name + "/passive_ds_stiffness",1);
    orient_pub    = nh.advertise<geometry_msgs::Quaternion>("/"     + name_space + "/" + controller_name + "/passive_ds_command_orient",1);

    stiff_msg.data.resize(3);

}

void Ros_passive_ds::sendCartVel(const geometry_msgs::Twist& twist_){
    velocity_pub.publish(twist_);
}

void Ros_passive_ds::sendStiff(const std_msgs::Float64MultiArray& stiff_msg){
    stiffness_pub.publish(stiff_msg);
}

void Ros_passive_ds::sendOrient(const geometry_msgs::Quaternion &orient_msg){
    orient_pub.publish(orient_msg);
}

}
