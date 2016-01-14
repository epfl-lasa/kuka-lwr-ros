#ifndef KUKA_ACTION_SERVER_BASE_EE_J_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_EE_J_ACTION_H_

/**
    Base End Effector Action

    Provides a default implementation of ros communicating protocols for the KUKA robot.

    Interface for joint_kinematics_imp.h ros controller

  **/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"


namespace asrv{

class Base_ee_j_action{

protected:

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Base_ee_j_action(ros::NodeHandle&   nh,const std::string& controller_name = "joint_kinematics_imp");

    void sendCartPose(const geometry_msgs::Pose & pose_msg);

    void sendCartVel(const geometry_msgs::Twist& twist_);

    void sendJointPos(const std_msgs::Float64MultiArray& joint_msg);

    void sendStiff(const std_msgs::Float64MultiArray& stiff_msg);

    void sendDamp(const std_msgs::Float64MultiArray& damp_msg);

    void sendGrav(const std_msgs::Bool& grav_msg);

private:

    void pose_callback(const geometry_msgs::PoseConstPtr& msg);

private:

    ros::Publisher                          joint_pub;
    ros::Publisher                          position_pub;
    ros::Publisher                          velocity_pub;
    ros::Publisher                          stiffness_pub;
    ros::Publisher                          damping_pub;
    ros::Publisher                          grav_pub;
    ros::Subscriber                         pose_sub;

protected:


    geometry_msgs::Pose                     ee_pos_msg;
    geometry_msgs::Twist                    ee_vel_msg;
    std_msgs::Float64MultiArray             joint_pos_msg;
    std_msgs::Float64MultiArray             stiff_msg;
    std_msgs::Float64MultiArray             damp_msg;

    std_msgs::Bool                          grav_msg;

    tf::Pose                                ee_pose_current;
    bool                                    b_received;

};

}
#endif
