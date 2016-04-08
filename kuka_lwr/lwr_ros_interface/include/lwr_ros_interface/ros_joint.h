#ifndef KUKA_ACTION_SERVER_BASE_J_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_J_ACTION_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <lwr_fri/JointStates.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "std_msgs/Float64MultiArray.h"

namespace ros_controller_interface{

class Ros_joint{

public:

    typedef enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7,
    }KUKA_PARAM;


public:

   Ros_joint(ros::NodeHandle&   nh, const std::string& controller_name = "joint_controllers");

   void sendJointPos(const Eigen::VectorXd& joint_position);

   void sendStiff(const std_msgs::Float64MultiArray& stiff_msg);

   void sendDamp(const std_msgs::Float64MultiArray& damp_msg);

private:

    void jStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

protected:

    volatile double                         joint_pos_0;
    Eigen::VectorXd                         joint_sensed;


private:

    ros::Subscriber                         joint_sensor_sub;


    ros::Publisher                          joint_cmd_pub;
    ros::Publisher                          joint_stiff_pub;
    ros::Publisher                          joint_damp_pub;

protected:

    std_msgs::Float64MultiArray             joint_cmd_msg;
    std_msgs::Float64MultiArray             joint_stiff_msg;
    std_msgs::Float64MultiArray             joint_damp_msg;

};

}

#endif
