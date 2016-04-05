#ifndef  CONTROLLERS_CARTESIAN_POSITION_H_
#define  CONTROLLERS_CARTESIAN_POSITION_H_

#include "utils/definitions.h"
#include "controllers/base_controllers.h"
#include <map>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include "controllers/change_ctrl_mode.h"
#include <geometry_msgs/Pose.h>

namespace controllers{

class Cartesian_position : public Base_controllers {

public:

    Cartesian_position(ros::NodeHandle &nh,controllers::Change_ctrl_mode& change_ctrl_mode);

    void update(const KDL::Frame& x_,const KDL::Jacobian& J_,KDL::JntArrayAcc& joint_des, const ros::Duration& period);

    void stop();

private:

    void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);

private:

    Change_ctrl_mode &change_ctrl_mode;

    KDL::Frame x_;		//current pose
    KDL::Frame x_des_;	//desired pose

    KDL::Frame P_err;	//position error for PID computation
    KDL::Frame I_err;	//position error for PID computation
    KDL::Frame D_err;	//position error for PID computation

    double Kp, Ki, Kd;

    KDL::Twist x_err_;

    Eigen::MatrixXd             J_pinv_;
    Eigen::Matrix<double,3,3>   skew_;

    struct quaternion_
    {
        KDL::Vector v;
        double a;
    } quat_curr_, quat_des_;

    KDL::Vector v_temp_;

    ros::Subscriber     sub_command_pose_;
    bool cmd_flag_,bFirst;

};


}

#endif
