#ifndef LWR_CARTESIAN_VELOCITY_H_
#define LWR_CARTESIAN_VELOCITY_H_

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64MultiArray.h>


#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/PoseStamped.h>

#include "utils/definitions.h"
#include "controllers/change_ctrl_mode.h"
#include "base_controllers.h"

#include <boost/shared_ptr.hpp>

namespace controllers
{

class Cartesian_velocity : public Base_controllers {

public:

    enum class CART_TYPE
    {
        VELOCITY_OPEN_LOOP      =   0,
        VELOCITY_PASSIVE_DS     =   1
    };

public:

    Cartesian_velocity(ros::NodeHandle &nh,
                        controllers::Change_ctrl_mode& change_ctrl_mode);

    void cart_vel_update(KDL::JntArray&             tau_cmd,
                             const KDL::JntArray&       K,
                             const KDL::JntArray&       D,
                             const KDL::Twist&          x_dot_,
                             const KDL::Jacobian&       J_,
                             const KDL::JntArrayAcc& joint_msr_);

    void stop();

private:

    void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);
    void command_grav_wrench(const geometry_msgs::WrenchConstPtr &msg);
    void command_stiffness(const std_msgs::Float64MultiArray &msg);
    void command_damping(const std_msgs::Float64MultiArray& msg); // To do: convert to ptr

private:

    Change_ctrl_mode &change_ctrl_mode;

    ros::Subscriber     sub_command_vel_;
    ros::Subscriber     sub_command_grav_wrench_;

    ros::Subscriber     sub_command_damping_;
    ros::Subscriber     sub_command_stiffness_;

    Eigen::Matrix<double,6,1>     grav_wrench_;
    Eigen::Matrix<double,6,6>     local_stiffness_;
    Eigen::Matrix<double,6,6>     local_damping_; //To Do add orientation

    Eigen::MatrixXd J_transpose_pinv_;
    Eigen::Matrix<double,7,1> qd, nullspace_torque;


    bool                bFirst;

    KDL::Twist                                              x_des_vel_;



};

}


#endif
