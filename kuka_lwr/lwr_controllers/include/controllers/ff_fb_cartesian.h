#ifndef FF_FB_CARTESIAN_H_
#define FF_FB_CARTESIAN_H_

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <lwr_controllers/FF_FB_plan.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include "utils/definitions.h"
#include "controllers/change_ctrl_mode.h"
#include "base_controllers.h"



namespace controllers
{

class FF_FB_cartesian : public Base_controllers {

public:

    FF_FB_cartesian(ros::NodeHandle &nh,
                        controllers::Change_ctrl_mode& change_ctrl_mode);

    void cart_ff_fb_update(KDL::JntArray& tau_cmd,
                KDL::JntArrayAcc& joint_des,
                const KDL::JntArrayAcc& q_msr,
                const KDL::JntArray& K,
                const KDL::JntArray& D,
                const ros::Duration& period);

    void stop();

private:

    void command_ff_fb(const lwr_controllers::FF_FB_planConstPtr &msg);

private:

    Change_ctrl_mode &change_ctrl_mode;

    ros::Subscriber             sub_command_ff_fb_;
    lwr_controllers::FF_FB_plan cur_plan;
    ros::Time                   time_at_start;
    ros::Time                   cur_time;
    geometry_msgs::Wrench       cur_ff;
    geometry_msgs::Pose         cur_xd;
    geometry_msgs::Twist        cur_xd_dot;
    std_msgs::Float64MultiArray cur_fb;
    bool                        bFirst;

};

}


#endif
