#ifndef KUKA_ACTION_SERVER_BASE_J_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_J_ACTION_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <kuka_fri_bridge/JointStates.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <armadillo>
#include "std_msgs/Float64MultiArray.h"

namespace asrv{

class Base_j_action{

public:

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

   Base_j_action(ros::NodeHandle&   nh);

   void update_position(const Eigen::VectorXd& joint_position);

   void update_stiffness(const Eigen::VectorXd& joint_stiffness);

   void update_damping(const Eigen::VectorXd& joint_damping);

private:

    void jStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

protected:

    Eigen::VectorXd                         joint_sensed;


private:

    ros::Subscriber                         joint_sensor_sub;


    ros::Publisher                          joint_cmd_pub;
    ros::Publisher                          joint_stiff_pub;
    ros::Publisher                          joint_damp_pub;

    std_msgs::Float64MultiArray             joint_cmd_msg;
    std_msgs::Float64MultiArray             joint_stiff_msg;
    std_msgs::Float64MultiArray             joint_damp_msg;

};

}

#endif
