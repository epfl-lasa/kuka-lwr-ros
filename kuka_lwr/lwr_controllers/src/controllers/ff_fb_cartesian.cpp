#include "controllers/ff_fb_cartesian.h"

namespace controllers{

FF_FB_cartesian::FF_FB_cartesian(ros::NodeHandle& nh,
                                      Change_ctrl_mode &change_ctrl_mode)
    : Base_controllers(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN),
      change_ctrl_mode(change_ctrl_mode), u_ff(6)
{
    sub_command_ff_fb_       = nh.subscribe("command_ff_fb_plan",      1, &FF_FB_cartesian::command_ff_fb,     this);
    bFirst = false;
    u_ff.setZero();
    ROS_INFO_STREAM("constructor [FF_FB]");
}

void FF_FB_cartesian::stop(){
    ROS_INFO_STREAM("stopping [FF_FB]");
    // Set to zero the ff and fb parts
    u_ff.setZero();
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

    u_ff[0] = cur_plan.ff[i].force.x;
    u_ff[1] = cur_plan.ff[i].force.y;
    u_ff[2] = cur_plan.ff[i].force.z;
    u_ff[3] = cur_plan.ff[i].torque.x;
    u_ff[4] = cur_plan.ff[i].torque.y;
    u_ff[5] = cur_plan.ff[i].torque.z;

    // Construct Eigen feedback matrix
    Eigen::MatrixXd K(6, 12);
    int ii = 0;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 12; ++j) {
            K(i, j) = cur_plan.fb[i].data[ii++];
        }
    }

    // Error vector for second order dynamics
    Eigen::VectorXd e(12);
    e << cur_plan.xd[i].position.x - x_.p.x(), cur_plan.xd[i].position.y - x_.p.y(), cur_plan.xd[i].position.z - x_.p.z() , Eigen::VectorXd::Zero(3) ,
         cur_plan.xd_dot[i].linear.x - x_dot_.vel.x() , cur_plan.xd_dot[i].linear.y - x_dot_.vel.y() , cur_plan.xd_dot[i].linear.z - x_dot_.vel.z() , Eigen::VectorXd::Zero(3);

    // Control law = J^T (u_ff + K (x_d - x))
    tau_cmd.data = J_.data.transpose() * (u_ff + K*e);
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
