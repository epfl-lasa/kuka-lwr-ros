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
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"


namespace ros_controller_interface{

inline void tf2msg(const tf::Vector3& pos,const tf::Quaternion& orient, geometry_msgs::Pose& pos_msg){
    pos_msg.position.x = pos.getX();
    pos_msg.position.y = pos.getY();
    pos_msg.position.z = pos.getZ();
    pos_msg.orientation.w = orient.w();
    pos_msg.orientation.x = orient.x();
    pos_msg.orientation.y = orient.y();
    pos_msg.orientation.z = orient.z();
}

class Ros_ee_j{

protected:

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Ros_ee_j(ros::NodeHandle&   nh,const std::string& controller_name = "joint_controllers");

    void sendCartPose(const geometry_msgs::Pose & pose_msg);

    void sendCartVel(const geometry_msgs::Twist& twist_);

    void sendJointPos(const std_msgs::Float64MultiArray& joint_msg);

    void sendStiff(const std_msgs::Float64MultiArray& stiff_msg);

    void sendDamp(const std_msgs::Float64MultiArray& damp_msg);

    void sendGrav(const std_msgs::Bool& grav_msg);

    void sendOrient(const geometry_msgs::Quaternion &orient_msg);

    void sendString(const std_msgs::String& string_msg);

private:

    void pose_callback(const geometry_msgs::PoseConstPtr& msg);

private:

    ros::Publisher                          joint_pub;
    ros::Publisher                          position_pub;
    ros::Publisher                          velocity_pub;
    ros::Publisher                          stiffness_pub;
    ros::Publisher                          damping_pub;
    ros::Publisher                          grav_pub;
    ros::Publisher                          orient_pub;
    ros::Publisher                          string_pub;
    ros::Subscriber                         pose_sub;

protected:


    geometry_msgs::Pose                     ee_pos_msg;
    geometry_msgs::Twist                    ee_vel_msg;
    std_msgs::Float64MultiArray             joint_pos_msg;
    std_msgs::Float64MultiArray             stiff_msg;
    std_msgs::Float64MultiArray             damp_msg;
    geometry_msgs::Quaternion               orient_msg;

    std_msgs::Bool                          grav_msg;
    std_msgs::String                        string_msg;

    tf::Pose                                ee_pose_current;
    bool                                    b_received;

};

}
#endif
