#include "controllers/open_loop_cartesian.h"

namespace controllers{

Open_loop_cartesian::Open_loop_cartesian(ros::NodeHandle& nh,
                                      Change_ctrl_mode &change_ctrl_mode,
                                     boost::shared_ptr<KDL::ChainIkSolverVel_pinv>& ik_vel_solver)
    : Base_controllers(lwr_controllers::CTRL_MODE::CART_VELOCITIY),
      change_ctrl_mode(change_ctrl_mode),
      ik_vel_solver_(ik_vel_solver)
{

    sub_command_vel_       = nh.subscribe("command_vel",      1, &Open_loop_cartesian::command_cart_vel,     this);
    sub_command_orient_    = nh.subscribe("command_orient",   1 ,&Open_loop_cartesian::command_orient,       this);
    sub_command_pose_      = nh.subscribe("command_pos",      1, &Open_loop_cartesian::command_cart_pos,     this);

    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    bFirst = false;

}

void Open_loop_cartesian::stop(){
    ROS_INFO_STREAM("stopping [CARTESIAN]");
    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    bFirst = false;
}

void Open_loop_cartesian::cart_vel_update(KDL::JntArray &tau_cmd, KDL::JntArrayAcc& joint_des, const KDL::JntArrayAcc& q_msr, const KDL::JntArray& K, const KDL::JntArray& D, const ros::Duration& period){
    ik_vel_solver_->CartToJnt(q_msr.q,x_des_vel_,joint_des.qdot);
    for (int i = 0; i < joint_des.q.data.size(); i++){
        // integrating q_dot -> getting q (Euler method)
        joint_des.q(i) = joint_des.q(i) + period.toSec()*joint_des.qdot(i);
        // joint position and velocity error to torque
        tau_cmd(i)     =   -D(i) * (q_msr.qdot(i) - joint_des.qdot(i)) - K(i) * (q_msr.q(i)  - joint_des.q(i));
    }
}

void Open_loop_cartesian::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO_THROTTLE(1.0,"command cart pos");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));
}

void Open_loop_cartesian::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    ROS_INFO_THROTTLE(1.0,"command_cart_vel");
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;
    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_VELOCITIY);
    }
    bFirst            = true;
}

void Open_loop_cartesian::command_orient(const geometry_msgs::Quaternion &msg){
    x_des_orient_.setX(msg.x);
    x_des_orient_.setY(msg.y);
    x_des_orient_.setZ(msg.z);
    x_des_orient_.setW(msg.w);
}





}
