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

    u_ff << cur_plan.ff[i].force.x , cur_plan.ff[i].force.y , cur_plan.ff[i].force.z, cur_plan.ff[i].torque.x, cur_plan.ff[i].torque.y, cur_plan.ff[i].torque.z;

    // Error vector for second order dynamics
    Eigen::VectorXd e(12);

    e << cur_plan.xd[i].position.x - x_.p.x(), cur_plan.xd[i].position.y - x_.p.y(), cur_plan.xd[i].position.z - x_.p.z(), 0, 0, 0,
         cur_plan.xd_dot[i].linear.x - x_dot_.vel.x() , cur_plan.xd_dot[i].linear.y - x_dot_.vel.y() , cur_plan.xd_dot[i].linear.z - x_dot_.vel.z() , 0, 0, 0;

    // Construct Eigen feedback matrix
    Eigen::MatrixXd K(6, 12);
    int ii = 0;
    for (int j = 0; j < 6; ++j) {
        for (int k = 0; k < 12; ++k) {
            K(j, k) = cur_plan.fb[i].data[ii++];
        }
    }

    // Control law = J^T (u_ff + K (x_d - x))
    Eigen::VectorXd u;
    u = u_ff + K*e;

    tau_cmd.data = J_.data.transpose() * u;

    //ROS_INFO_THROTTLE(0.002,"torque ->  %f, %f, %f, %f, %f, %f\n", tau_cmd.data[0], tau_cmd.data[1],tau_cmd.data[2],tau_cmd.data[3],tau_cmd.data[4],tau_cmd.data[5],tau_cmd.data[6]);
    //ROS_INFO_THROTTLE(0.002,"u_ff -> %f, %f, %f, %f, %f, %f\n", u_ff[0], u_ff[1], u_ff[2], u_ff[3], u_ff[4], u_ff[5]);
    //ROS_INFO_THROTTLE(0.002,"u -> %f, %f, %f, %f, %f, %f\n", u[0], u[1], u[2], u[3], u[4], u[5]);
    //ROS_INFO_THROTTLE(0.002,"error at sample -> %d: %f, %f, %f, %f, %f, %f\n", i, cur_plan.xd[i].position.x- x_.p.x(), cur_plan.xd[i].position.y- x_.p.y(), cur_plan.xd[i].position.z- x_.p.z(), e(6), e(7), e(8));
}


void FF_FB_cartesian::command_ff_fb(const lwr_controllers::FF_FB_planConstPtr &msg){
  ROS_INFO_THROTTLE(1.0,"command_ff_fb");

    // Copy the whole feedback plan
    cur_plan.times = msg->times;
    cur_plan.ff = msg->ff;
    cur_plan.xd = msg->xd;
    cur_plan.xd_dot = msg->xd_dot;
    cur_plan.fb = msg->fb;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::FF_FB_CARTESIAN);
    }
    bFirst            = true;
}


}
