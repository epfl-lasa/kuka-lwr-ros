#include "controllers/cartesian_velocity.h"
#include <utils/pseudo_inversion.h>

namespace controllers{

Cartesian_velocity::Cartesian_velocity(ros::NodeHandle& nh,
                                       Change_ctrl_mode& change_ctrl_mode)
    : Base_controllers(lwr_controllers::CTRL_MODE::CART_VELOCITIY),
      change_ctrl_mode(change_ctrl_mode)
{
    sub_command_vel_ = nh.subscribe("command_vel", 1,
                                    &Cartesian_velocity::command_cart_vel,
                                    this, ros::TransportHints().reliable().tcpNoDelay());
    sub_command_grav_wrench_ = nh.subscribe("command_grav_wrench", 1,
                                            &Cartesian_velocity::command_grav_wrench,
                                            this, ros::TransportHints().reliable().tcpNoDelay());

    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    grav_wrench_.setZero();

    qd << 0.022273175418376923, 0.6135804057121277, 0.0058699785731732845,
        -1.7890702486038208, -0.008821399882435799, -0.8749043345451355, 0.010746323503553867;

    bFirst = false;
}

void Cartesian_velocity::stop(){
    ROS_INFO_STREAM("stopping [CARTESIAN VELOCITY]");
    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    bFirst = false;
}


void Cartesian_velocity::cart_vel_update(KDL::JntArray&             tau_cmd,
                                         const KDL::JntArray&       D,
                                         const KDL::JntArray&       K_vel,
                                         const KDL::Twist&          x_dot_,
                                         const KDL::Jacobian&       J_,
                                         const KDL::JntArrayAcc& joint_msr_)
{
    // Tracking error
    Eigen::VectorXd force_ee(6), q_d(7);

    force_ee(0) = K_vel(0)*(x_des_vel_.vel(0) - x_dot_.vel(0));
    force_ee(1) = K_vel(1)*(x_des_vel_.vel(1) - x_dot_.vel(1));
    force_ee(2) = K_vel(2)*(x_des_vel_.vel(2) - x_dot_.vel(2));
    force_ee(3) = K_vel(3)*x_des_vel_.rot(0) - D(3)*x_dot_.rot(0);
    force_ee(4) = K_vel(4)*x_des_vel_.rot(1) - D(4)*x_dot_.rot(1);
    force_ee(5) = K_vel(5)*x_des_vel_.rot(2) - D(5)*x_dot_.rot(2);

    // Compensate for gravity
    force_ee  << force_ee + grav_wrench_;

    std::cout << " Force: " << force_ee.transpose() << std::endl;
    std::cout << "Grav Wrench:" << grav_wrench_.transpose() << std::endl;

    // Regulate around a good joint configuration in the nullspace
    // Compute pseudoinverse (Use mass matrix pseudoinverse to cancel out
    // correctly the accelerations, see Khatib 1987)

//    std::cout << "Until here!";
    pseudo_inverse(J_.data.transpose(), J_transpose_pinv_);

    //std::cout << "Jacobian: " << std::endl << J_.data.transpose() << std::endl;
    //std::cout << "J_pinv_: " << std::endl << J_transpose_pinv_<< std::endl;

//    std::cout << "Until here!!";
    //nullspace_torque << (Eigen::MatrixXd::Identity(7, 7) - J_.data.transpose()*J_transpose_pinv_)*(2.0*(qd - joint_msr_.q.data) - 0.01*joint_msr_.qdot.data);
    //std::cout << "Nullspace torque: " << std::endl << nullspace_torque << std::endl;

    tau_cmd.data = J_.data.transpose() * force_ee ;//+ nullspace_torque;
}


void Cartesian_velocity::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
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

void Cartesian_velocity::command_grav_wrench(const geometry_msgs::WrenchConstPtr &msg){
    grav_wrench_ << msg->force.x, msg->force.y, msg->force.z,
                    msg->torque.x, msg->torque.y, msg->torque.z;
    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_VELOCITIY);
    }
    bFirst            = true;
}





}
