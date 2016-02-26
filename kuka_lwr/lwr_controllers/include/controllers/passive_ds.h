#ifndef PASSIVE_DS_H_
#define PASSIVE_DS_H_

#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <lwr_controllers/passive_ds_paramConfig.h>
#include <passive_ds_controller.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Eigen>
#include <lwr_controllers/rot_stiffnessConfig.h>
#include <dynamic_reconfigure/server.h>

namespace controllers{

class Passive_ds{

public:

    Passive_ds(ros::NodeHandle& nh);

    void stop();

    void update();

private:

    void ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level);

    void rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level);

private:

    Eigen::VectorXd dx_msr_;
    Vec             dx_linear_des_;
    Vec             dx_linear_msr_;
    Vec             F_linear_des_;     // desired linear force
    Eigen::VectorXd F_ee_des_;         // desired end-effector force

    tf::Matrix3x3       x_des_orient_rot_;
    tf::Matrix3x3       x_orient_;
    double              rot_stiffness;
    boost::scoped_ptr<DSController>                     passive_ds_controller;
    boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig> >      dynamic_server_ds_param;
    boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig> >         dynamic_server_rot_stiffness_param;

    ros::NodeHandle nd5, nd6;

    tf::Matrix3x3    err_orient;

};

}

#endif
