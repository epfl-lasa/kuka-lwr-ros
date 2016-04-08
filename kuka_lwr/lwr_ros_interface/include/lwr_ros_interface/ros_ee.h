#ifndef KUKA_ACTION_SERVER_BASE_EE_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_EE_ACTION_H_

/**
    Base End Effector Action

    Provides a default implementation of ros communicating protocols for the KUKA robot.
    A publisher and subscriber are implemented to read the robot's end effector state
    and command the end effectors state.

  **/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "std_msgs/Float64MultiArray.h"


namespace ros_controller_interface{

class Ros_ee{

private:

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Ros_ee(ros::NodeHandle&   nh);

    void sendPose(const geometry_msgs::Pose & pose_msg);

    void sendVel(const geometry_msgs::Twist& twist_);

    void sendStiff(const std_msgs::Float64MultiArray& stiff_msg);

    void sendDamp(const std_msgs::Float64MultiArray& damp_msg);

private:

    void pose_callback(const geometry_msgs::PoseConstPtr& msg);

private:

    ros::Publisher                          position_pub;
    ros::Publisher                          velocity_pub;
    ros::Publisher                          stiffness_pub;
    ros::Publisher                          damping_pub;
    ros::Subscriber                         pose_sub;

protected:

    geometry_msgs::Pose                     ee_pos_msg;
    geometry_msgs::Twist                    ee_vel_msg;
    std_msgs::Float64MultiArray             ee_stiff_msg;
    std_msgs::Float64MultiArray             ee_damp_msg;

    tf::Pose                                ee_pose_current;
    bool                                    b_received;

};

}
#endif
