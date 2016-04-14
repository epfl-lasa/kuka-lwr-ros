#include "controllers/ff_fb_cartesian.h"

namespace controllers{

FF_FB_cartesian::FF_FB_cartesian(ros::NodeHandle& nh,
                                      Change_ctrl_mode &change_ctrl_mode)
    : Base_controllers(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN),
      change_ctrl_mode(change_ctrl_mode), F_ee_des(6)
{
    sub_command_ff_fb_       = nh.subscribe("command_ff_fb_plan",      1, &FF_FB_cartesian::command_ff_fb,     this);
    bFirst = false;
    F_ee_des.setZero();
    ROS_INFO_STREAM("constructor [FF_FB]");
}

void FF_FB_cartesian::stop(){
    ROS_INFO_STREAM("stopping [FF_FB]");
    // Set to zero the ff and fb parts
    F_ee_des.setZero();
    bFirst = false;
}


void FF_FB_cartesian::update(KDL::JntArray &tau_cmd, const KDL::Frame& x_, const KDL::Twist& x_dot_, const KDL::Jacobian& J_){
    ros::Time time_now = ros::Time::now();
    int i;

    // Get the corresponding sample of the FF FB trajectory
    for (i = 0 ; i < (cur_plan.times.size()-2); i++) {
        if ( cur_plan.times[i].data > time_now ) {
            if (i>0) {
                i--;
            }
            break;
        }
    }

    ROS_INFO_THROTTLE(1.0,"Exert force %f, %f, %f\n", cur_plan.ff[i].force.x, cur_plan.ff[i].force.y, cur_plan.ff[i].force.z);

    F_ee_des[0] = cur_plan.ff[i].force.x;
    F_ee_des[1] = cur_plan.ff[i].force.y;
    F_ee_des[2] = cur_plan.ff[i].force.z;
    F_ee_des[3] = cur_plan.ff[i].torque.x;
    F_ee_des[4] = cur_plan.ff[i].torque.y;
    F_ee_des[5] = cur_plan.ff[i].torque.z;

    // TODO: Compute feedback
    // Compute error
    // e = cur_plan.xd - x_;
    // e_dot = cur_plan.xd_dot - x_dot_;
    // F_ee_des_+= cur_plan.fb ([e e_dot]);

    tau_cmd.data = J_.data.transpose() * F_ee_des;
}


void FF_FB_cartesian::command_ff_fb(const lwr_controllers::FF_FB_planConstPtr &msg){
  ROS_INFO_THROTTLE(1.0,"command_ff_fb");

    // Copy the whole feedback plan
    cur_plan.times = msg->times;
    cur_plan.ff = msg->ff;
    cur_plan.fb = msg->fb;
    cur_plan.xd = msg->xd;
    cur_plan.xd_dot = msg->xd_dot;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN);
    }
    bFirst            = true;
}


}
