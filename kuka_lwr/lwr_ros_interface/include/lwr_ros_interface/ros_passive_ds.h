#ifndef KUKA_ROS_CONTROLLER_INTERFACE_PASSIVE_DS_H_
#define KUKA_ROS_CONTROLLER_INTERFACE_PASSIVE_DS_H_


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64MultiArray.h"

namespace ros_controller_interface{


class Ros_passive_ds{

public:

    Ros_passive_ds(ros::NodeHandle&   nh, const std::string& robot_name_space = "lwr",const std::string& controller_name = "joint_controllers");


public:

    void sendCartVel(const geometry_msgs::Twist& twist_);

    void sendStiff(const std_msgs::Float64MultiArray& stiff_msg);

    void sendOrient(const geometry_msgs::Quaternion &orient_msg);

private:

    ros::Publisher                          velocity_pub;
    ros::Publisher                          stiffness_pub;
    ros::Publisher                          orient_pub;

public:

    geometry_msgs::Twist                    ee_vel_msg;
    std_msgs::Float64MultiArray             stiff_msg;
    geometry_msgs::Quaternion               orient_msg;

};

}



#endif
