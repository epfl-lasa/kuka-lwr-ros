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
#include <dynamic_reconfigure/server.h>

#include <kdl/jntarray.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayacc.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include "utils/pseudo_inversion.h"
#include <qpOASES.hpp>

namespace controllers{

class Passive_ds  : public Base_controllers {


public:

    Passive_ds(ros::NodeHandle& nh,controllers::Change_ctrl_mode& change_ctrl_mode);

    void stop();

    // void update(KDL::JntArray& tau_cmd, const KDL::Jacobian&  J, const KDL::JntArrayAcc& joint_msr_, const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_, const KDL::Vector &p = KDL::Vector());
    void update(KDL::Wrench &wrench, KDL::JntArray& tau_cmd, const KDL::Jacobian&  J, const KDL::JntArrayAcc& joint_msr_, const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_, const KDL::Vector &p, Eigen::MatrixXd inertiaMatrix,  Eigen::VectorXd coriolis);

private:

    /// Dynamic reconfigure

    void ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level);

private:

    /// ROS topic callbacks

    void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);

    void command_wrench(const geometry_msgs::WrenchConstPtr& msg);

    void command_orient(const geometry_msgs::Quaternion &msg);

    void command_damping_eig(const std_msgs::Float64MultiArray& msg);

    void command_rot_stiff(const std_msgs::Float64& msg);

    void command_rot_damp(const std_msgs::Float64& msg);

    void publish_open_loop_pos(const ros::Duration &period, const ros::Time& time);

    void command_nullspace(const std_msgs::Float32MultiArray& msg);
private:

    /// Ctrl mode
    Change_ctrl_mode&   change_ctrl_mode;
    bool                bFirst;
    bool                bDebug;
    bool                bSmooth;


    /// DS parameters
    /// Linear
    Eigen::VectorXd     dx_msr_;
    Vec                 dx_linear_des_;
    Vec                 dx_angular_des_;
    Vec                 dx_linear_msr_;
    Vec                 dx_angular_msr_;
    Vec                 F_linear_des_;     // desired linear force
    Eigen::VectorXd     wrench_des_;
    Eigen::VectorXd     F_ee_des_;         // desired end-effector force
    /// Rotation
    KDL::Rotation         err_orient;
    tf::Vector3           err_orient_axis;
    double                err_orient_angle;
    tf::Vector3           torque_orient;
    double                qx, qy, qz, qw;
    tf::Quaternion        q;

    Eigen::Matrix3f _damping;

    double              smooth_val_;
    double              rot_stiffness;
    double              rot_damping;

    bool _useNullSpace;
    double _jointLimitsGain;
    double _desiredJointsGain;
    double _jointVelocitiesGain;
    double _wrenchGain;
    double _nullspaceCommandGain;

    Eigen::Matrix<float,7,1> _torqueLimits;
    Eigen::Matrix<float,7,1> _jointVelocityLimits;
    Eigen::Matrix<float,7,1> _jointLimits;
    
    boost::scoped_ptr<DSController>                     passive_ds_controller;

    /// Dynamic reconfigure

    boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig> >      dynamic_server_ds_param;
    ros::NodeHandle nd5;


    qpOASES::SQProblem *_sqp;
  // qpOASES uses row-major storing
    qpOASES::real_t H_qp[14*14];
    qpOASES::real_t A_qp[7*14];
    qpOASES::real_t g_qp[14];
    qpOASES::real_t lb_qp[14];
    qpOASES::real_t ub_qp[14];
    qpOASES::real_t lbA_qp[7];
    qpOASES::real_t ubA_qp[7];

private:
    /// ROS topic
    KDL::Twist             x_des_vel_;
    KDL::Rotation          rot_des_;

    ros::Subscriber         sub_command_vel_;
    ros::Subscriber         sub_command_force_;
    ros::Subscriber         sub_command_orient_;
    ros::Subscriber         sub_eig_;
    ros::Subscriber         sub_stiff_;
    ros::Subscriber         sub_damp_;
    ros::Subscriber         sub_command_nullspace_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_damping_matrix_;


    bool _initQP;

    /// ROS debug

    ros::Publisher                  pub_F_;
    ros::Publisher                  torque_pub_;
    std_msgs::Float64MultiArray     F_msg_,tau_msg_;
    lwr_controllers::passive_ds_paramConfig config_cfg;

    // null-spae control
    Eigen::Matrix<double,7,1> qd, nullspace_torque, nullspace_command;

    bool _useQP;
    bool _useKDLInertiaMatrix;
    bool _useCoriolis;
    double _alpha1;
    double _alpha2;
    double _alpha3;



};

}

#endif
