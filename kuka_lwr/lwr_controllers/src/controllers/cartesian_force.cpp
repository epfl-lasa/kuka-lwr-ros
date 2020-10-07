#include "controllers/cartesian_force.h"

namespace controllers {
    Cartesian_force::Cartesian_force(ros::NodeHandle& nh, controllers::Change_ctrl_mode& change_ctrl_mode) :
        Base_controllers(lwr_controllers::CTRL_MODE::CART_FORCE),
        change_ctrl_mode(change_ctrl_mode)
    {
        sub_command_force_ = nh.subscribe("command_wrench", 1, &Cartesian_force::command_cart_force, this, ros::TransportHints().reliable().tcpNoDelay());
        F_ee_des_.resize(6);
        F_ee_des_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
        bFirst = false;
    }

    void Cartesian_force::stop()
    {
        ROS_INFO_STREAM("stopping [CART_FORCE]");
        F_ee_des_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
        bFirst = false;
    }

    void Cartesian_force::update(KDL::JntArray& tau_cmd, const KDL::Jacobian& J)
    {
        tau_cmd.data = J.data.transpose() * F_ee_des_;
    }

    void Cartesian_force::command_cart_force(const geometry_msgs::WrenchConstPtr& msg)
    {
        F_ee_des_(0) = msg->force.x;
        F_ee_des_(1) = msg->force.y;
        F_ee_des_(2) = msg->force.z;
        F_ee_des_(3) = msg->torque.x;
        F_ee_des_(4) = msg->torque.y;
        F_ee_des_(5) = msg->torque.z;

        if (!bFirst) {
            change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_FORCE);
        }
        bFirst = true;
        ROS_INFO("CART_FORCE: Command received");
    }

} // namespace controllers
