#ifndef CARTESIAN_FORCE_H
#define CARTESIAN_FORCE_H

#include "base_controllers.h"
#include "controllers/change_ctrl_mode.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <kdl/jntarray.hpp>
#include <geometry_msgs/Wrench.h>

namespace controllers {
    class Cartesian_force  : public Base_controllers {
    public:
        Cartesian_force(ros::NodeHandle& nh, controllers::Change_ctrl_mode& change_ctrl_mode);

        void stop() override;

        void update(KDL::JntArray& tau_cmd, const KDL::Jacobian&  J);

    private:
        /// ROS topic callback
        void command_cart_force(const geometry_msgs::WrenchConstPtr& msg);

        /// Ctrl mode
        Change_ctrl_mode&   change_ctrl_mode;
        bool                bFirst;

        /// ROS topic
        ros::Subscriber     sub_command_force_;

        Eigen::VectorXd     F_ee_des_;         // desired end-effector force and torque
    };
}

#endif
