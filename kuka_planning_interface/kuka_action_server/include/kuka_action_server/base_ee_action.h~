#ifndef KUKA_ACTION_SERVER_BASE_EE_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_EE_ACTION_H_

/**
    Base End Effector Action

    Provides a default implementation of ros communicating protocols for the KUKA robot.
    A publisher and subscriber are implemented to read the robot's end effector state
    and command the end effectors state.

  **/

#include <ros/ros.h>
//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "lwr_controllers/PoseRPY.h"

//#include "MathLib/MathLib.h"
#include <armadillo>

#include <tf/LinearMath/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace asrv{

class Base_ee_action{

public:

    Base_ee_action(ros::NodeHandle&   nh);

    void sendPose(const geometry_msgs::Pose & pose_msg);

    void sendVel(const geometry_msgs::Twist& twist_);

private:

    void pose_callback(const geometry_msgs::PoseConstPtr& msg);

private:

    // ros::Subscriber                     sub_, sub_ft_;
    ros::Publisher                        position_pub;//pub_, pub_ft_, pub_vel_;
    ros::Publisher                        velocity_pub;
    ros::Subscriber                       pose_sub;
    tf::Matrix3x3                         tmp;


protected:

    geometry_msgs::Pose             ee_pos_msg;
    geometry_msgs::Twist            ee_vel_msg;
    tf::Pose                        ee_pose_current;
    bool                            b_received;

};

}
#endif
