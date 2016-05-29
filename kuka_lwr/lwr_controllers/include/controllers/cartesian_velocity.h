#ifndef LWR_CARTESIAN_VELOCITY_H_
#define LWR_CARTESIAN_VELOCITY_H_

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

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
                        controllers::Change_ctrl_mode& change_ctrl_mode,
                        boost::shared_ptr<KDL::ChainIkSolverVel_pinv>& ik_vel_solver);

    void cart_vel_update(KDL::JntArray& tau_cmd,
                KDL::JntArrayAcc& joint_des,
                const KDL::JntArrayAcc& q_msr,
                const KDL::JntArray& K,
                const KDL::JntArray& D,
                const ros::Duration& period,
                const ros::Time &time);

    void stop();

private:

    void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);

    void command_orient(const geometry_msgs::Quaternion &msg);

    void publish_open_loop_pos(const ros::Duration &period, const ros::Time& time);

private:

    Change_ctrl_mode &change_ctrl_mode;

    ros::Subscriber     sub_command_vel_;
    ros::Subscriber     sub_command_orient_;

    tf::Quaternion      x_des_orient_;
    tf::Matrix3x3       x_des_orient_rot_;
    tf::Matrix3x3       x_orient_;
    double              rot_stiffness;
    bool                bFirst;

    boost::shared_ptr<KDL::ChainIkSolverVel_pinv>           ik_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive>      fk_pos_solver;
    KDL::Twist                                              x_des_vel_;



};

}


#endif
