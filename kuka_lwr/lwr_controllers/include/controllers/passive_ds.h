#ifndef PASSIVE_DS_H_
#define PASSIVE_DS_H_

#include "base_controllers.h"
#include "controllers/change_ctrl_mode.h"

#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <lwr_controllers/passive_ds_paramConfig.h>
#include <passive_ds_controller.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Eigen>
#include <lwr_controllers/rot_stiffnessConfig.h>
#include <dynamic_reconfigure/server.h>

#include <kdl/jntarray.hpp>
#include <kdl/framevel.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

namespace controllers{

class Passive_ds  : public Base_controllers {

public:

    Passive_ds(ros::NodeHandle& nh,controllers::Change_ctrl_mode& change_ctrl_mode);

    void stop();

    void update(KDL::JntArray& tau_cmd, const KDL::Jacobian&  J, const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_);

private:

    /// Dynamic reconfigure

    void ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level);

    void rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level);


private:

    /// ROS topic callbacks

    void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);

    void command_orient(const geometry_msgs::Quaternion &msg);

    void command_damping_eig(const std_msgs::Float64MultiArray& msg);

    void command_rot_stiff(const std_msgs::Float64& msg);

    void publish_open_loop_pos(const ros::Duration &period, const ros::Time& time);

private:

    /// Ctrl mode
    Change_ctrl_mode&   change_ctrl_mode;
    bool                bFirst;


    /// DS parameters
    /// Linear
    Eigen::VectorXd     dx_msr_;
    Vec                 dx_linear_des_;
    Vec                 dx_linear_msr_;
    Vec                 dx_angular_msr_;
    Vec                 F_linear_des_;     // desired linear force
    Eigen::VectorXd     F_ee_des_;         // desired end-effector force
    /// Rotation
  //  tf::Matrix3x3       x_des_orient_rot_;
 //   tf::Matrix3x3       x_orient_des_;
   // tf::Matrix3x3       err_orient;
    KDL::Rotation         err_orient;
    tf::Vector3           err_orient_axis;
    double                err_orient_angle;
    tf::Vector3           torque_orient;
    double                qx, qy, qz, qw;


    double              rot_stiffness;

    boost::scoped_ptr<DSController>                     passive_ds_controller;

    /// Dynamic reconfigure

    boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig> >      dynamic_server_ds_param;
    boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig> >         dynamic_server_rot_stiffness_param;
    ros::NodeHandle nd5, nd6;



private:
    /// ROS topic
    KDL::Twist             x_des_vel_;
    KDL::Frame             x_des_;

    ros::Subscriber         sub_command_vel_;
    ros::Subscriber         sub_command_orient_;


    /// ROS debug

    ros::Publisher                  pub_F_;
    ros::Publisher                  torque_pub_;
    std_msgs::Float64MultiArray     F_msg_,tau_msg_;

};

}

#endif
