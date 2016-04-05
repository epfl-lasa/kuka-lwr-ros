#include "controllers/ff_fb_cartesian.h"

namespace controllers{

FF_FB_cartesian::FF_FB_cartesian(ros::NodeHandle& nh,
                                      Change_ctrl_mode &change_ctrl_mode)
    : Base_controllers(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN),
      change_ctrl_mode(change_ctrl_mode)
{
    sub_command_ff_fb_       = nh.subscribe("command_vel",      1, &FF_FB_cartesian::command_ff_fb,     this);

//    x_des_vel_.vel.Zero();
//    x_des_vel_.rot.Zero();
    bFirst = false;

}

void FF_FB_cartesian::stop(){
    ROS_INFO_STREAM("stopping [FF_FB]");
    // Set to zero the ff and fb parts
    bFirst = false;
}

// This is the actual controller
void FF_FB_cartesian::cart_ff_fb_update(KDL::JntArray &tau_cmd, KDL::JntArrayAcc& joint_des, const KDL::JntArrayAcc& q_msr, const KDL::JntArray& K, const KDL::JntArray& D, const ros::Duration& period){
//    fk_vel_solver_->CartToJnt(q_msr.q,x_des_vel_,joint_des.qdot);
//    for (int i = 0; i < joint_des.q.data.size(); i++){
//        // integrating q_dot -> getting q (Euler method)
//        joint_des.q(i) = joint_des.q(i) + period.toSec()*joint_des.qdot(i);
//        // joint position and velocity error to torque
//        tau_cmd(i)     =   -D(i) * (q_msr.qdot(i) - joint_des.qdot(i)) - K(i) * (q_msr.q(i)  - joint_des.q(i));
//    }
}


void FF_FB_cartesian::command_ff_fb(const lwr_controllers::FF_FB_planConstPtr &msg){
    ROS_INFO_THROTTLE(1.0,"command_ff_fb");

    // Copy the whole feedback plan
    cur_plan.times = msg->times;
    cur_plan.ff = msg->ff;
    cur_plan.fb = msg->fb;
    cur_plan.xd = msg->xd;
    cur_plan.xd_dot = msg->xd_dot;

    // Store the starting time
    time_at_start = ros::Time::now();

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN);
    }
    bFirst            = true;
}


}
