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

#include "eigen_conversions/eigen_msg.h"

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

    void update(KDL::JntArray &tau_cmd, const KDL::Frame& x_, const KDL::Twist& x_dot_, const KDL::Jacobian& J_);

    void stop();

private:

    void command_ff_fb(const lwr_controllers::FF_FB_planConstPtr &msg);

private:

    Change_ctrl_mode &change_ctrl_mode;

    ros::Subscriber             sub_command_ff_fb_;
    lwr_controllers::FF_FB_plan cur_plan;
    bool                        bFirst;

    Eigen::VectorXd             u_ff;         // desired end-effector force

};

}


#endif
