#ifndef KUKA_ROS_CONTROLLER_INTERFACE_PASSIVE_DS_H_
#define KUKA_ROS_CONTROLLER_INTERFACE_PASSIVE_DS_H_


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

namespace ros_controller_interface{


class Ros_passive_ds{

public:

    Ros_passive_ds(ros::NodeHandle&   nh, const std::string& robot_name_space = "lwr",const std::string& controller_name = "joint_controllers");


public:

    void sendDampEig(const std_msgs::Float64MultiArray& msg);

    void sendCartVel(const geometry_msgs::Twist& twist_);

    void sendRotStiff(const std_msgs::Float64& stiff_msg);

    void sendRotDamp(const std_msgs::Float64& damp_msg);

    void sendOrient(const geometry_msgs::Quaternion &orient_msg);

private:

    ros::Publisher                          velocity_pub;
    ros::Publisher                          rot_stiffness_pub;
    ros::Publisher                          rot_damp_pub;
    ros::Publisher                          orient_pub;
    ros::Publisher                          passive_ds_eig_pub;

public:

    geometry_msgs::Twist                    ee_vel_msg;
    std_msgs::Float64                       stiff_msg;
    std_msgs::Float64                       damp_msg;
    std_msgs::Float64MultiArray             eig_msg;
    geometry_msgs::Quaternion               orient_msg;


};

}



#endif
