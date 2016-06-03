#include "controllers/passive_ds.h"
#include <tf/transform_broadcaster.h>
#include <control_toolbox/filters.h>

namespace controllers{

static const double thrott_time = 3.0;

Passive_ds::Passive_ds(ros::NodeHandle &nh, controllers::Change_ctrl_mode &change_ctrl_mode):
    Base_controllers(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS),
    change_ctrl_mode(change_ctrl_mode)
{

    /// ROS topic

    sub_command_vel_       = nh.subscribe("passive_ds_command_vel",      1, &Passive_ds::command_cart_vel,     this );
    sub_command_orient_    = nh.subscribe("passive_ds_command_orient",   1 ,&Passive_ds::command_orient,       this );
    sub_eig_               = nh.subscribe("passive_ds_eig",              1 ,&Passive_ds::command_damping_eig,  this );
    sub_stiff_             = nh.subscribe("passive_ds_stiffness",        1 ,&Passive_ds::command_rot_stiff,    this );
    sub_damp_              = nh.subscribe("passive_ds_damping",          1 ,&Passive_ds::command_rot_damp,     this );

    /// Passive dynamical system

    passive_ds_controller.reset(new DSController(3,50.0,50.0));

    nd5 = ros::NodeHandle("ds_param");

    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));


    dynamic_server_ds_param->setCallback(    boost::bind(&Passive_ds::ds_param_callback,     this, _1, _2));
    dynamic_server_ds_param->getConfigDefault(config_cfg);

    for(std::size_t i = 0; i < 9; i++){
        err_orient.data[i] = 0;
    }

    dx_linear_des_.resize(3);
    dx_linear_msr_.resize(3);
    dx_angular_msr_.resize(3);

    F_ee_des_.resize(6);
    bFirst = false;

    rot_stiffness = config_cfg.rot_stiffness;
    rot_damping   = config_cfg.rot_damping;

    rot_des_ = KDL::Rotation::RPY(0,0,0);

    /// ROS pub debug

    pub_F_                 = nh.advertise<std_msgs::Float64MultiArray>("F_ee",10);
    torque_pub_            = nh.advertise<std_msgs::Float64MultiArray>("tau_pds",10);
    F_msg_.data.resize(6);
    tau_msg_.data.resize(7);

    bDebug      = config_cfg.debug;
    bSmooth     = config_cfg.bSmooth;
    smooth_val_ = config_cfg.smooth_val;
}


void Passive_ds::stop(){
    ROS_INFO_STREAM("stopping [PASSIVE_DS]");
    bFirst      = false;
}

void Passive_ds::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
    rot_stiffness = config.rot_stiffness;
    rot_damping   = config.rot_damping;
    bDebug        = config.debug;
    bSmooth       = config.bSmooth;
    smooth_val_   = config.smooth_val;
    config_cfg    = config;
}


void Passive_ds::update(KDL::JntArray& tau_cmd, const KDL::Jacobian &J, const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_, const KDL::Vector& p){


    F_ee_des_.setZero();

    /// set desired linear velocity
    dx_linear_des_(0)   = x_des_vel_(0);
    dx_linear_des_(1)   = x_des_vel_(1);
    dx_linear_des_(2)   = x_des_vel_(2);


    /// set measured linear and angular velocity
    if(bSmooth)
    {
        dx_linear_msr_(0)   = x_msr_vel_.vel(0);
        dx_linear_msr_(1)   = x_msr_vel_.vel(1);
        dx_linear_msr_(2)   = x_msr_vel_.vel(2);

        dx_angular_msr_(0)  = x_msr_vel_.rot(0);
        dx_angular_msr_(1)  = x_msr_vel_.rot(1);
        dx_angular_msr_(2)  = x_msr_vel_.rot(2);
    }else{
       dx_linear_msr_(0)    = filters::exponentialSmoothing(x_msr_vel_.vel(0), dx_linear_msr_(0),smooth_val_);
       dx_linear_msr_(1)    = filters::exponentialSmoothing(x_msr_vel_.vel(1), dx_linear_msr_(2),smooth_val_);
       dx_linear_msr_(2)    = filters::exponentialSmoothing(x_msr_vel_.vel(2), dx_linear_msr_(1),smooth_val_);

       dx_angular_msr_(0)   = filters::exponentialSmoothing(x_msr_vel_.rot(0), dx_angular_msr_(0),smooth_val_);
       dx_angular_msr_(1)   = filters::exponentialSmoothing(x_msr_vel_.rot(1), dx_angular_msr_(2),smooth_val_);
       dx_angular_msr_(2)   = filters::exponentialSmoothing(x_msr_vel_.rot(2), dx_angular_msr_(1),smooth_val_);
    }


    // ----------------- Linear velocity -> Force -----------------------//

    passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)

    F_ee_des_(0) = F_linear_des_(0);
    F_ee_des_(1) = F_linear_des_(1);
    F_ee_des_(2) = F_linear_des_(2);

    // ----------------- Debug -----------------------//

    if(bDebug){
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(p(0),p(1),p(2)) );
            rot_msr_.GetQuaternion(qx,qy,qz,qw);
            q = tf::Quaternion(qx,qy,qz,qw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rot_msr_"));
        }
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(p(0),p(1),p(2)) );
            rot_des_.GetQuaternion(qx,qy,qz,qw);
            q = tf::Quaternion(qx,qy,qz,qw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rot_target_"));
        }
    }

    // ----------------- Rotation target -> Force -----------------------//

    // damp any rotational motion
    err_orient = rot_msr_ * rot_des_.Inverse();
    err_orient.GetQuaternion(qx,qy,qz,qw);
    q = tf::Quaternion(qx,qy,qz,qw);

    err_orient_axis = q.getAxis();
    err_orient_angle = q.getAngle();

    if(bDebug){
        ROS_WARN_STREAM_THROTTLE(2.0,"err_orient_axis: " << err_orient_axis.getX() << " " << err_orient_axis.getY() << " " << err_orient_axis.getZ() );
        ROS_WARN_STREAM_THROTTLE(2.0,"err_orient_angle: " << err_orient_angle);
    }

    if(std::isnan(err_orient_angle) || std::isinf(err_orient_angle)){
        ROS_WARN_STREAM_THROTTLE(thrott_time,"err_orient_angle: " << err_orient_angle );
        err_orient_angle = 0;
    }

    // rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    torque_orient = err_orient_axis * err_orient_angle * (-rot_stiffness);

    F_ee_des_(3) = -rot_damping * dx_angular_msr_(0) + torque_orient.getX();
    F_ee_des_(4) = -rot_damping * dx_angular_msr_(1) + torque_orient.getY();
    F_ee_des_(5) = -rot_damping * dx_angular_msr_(2) + torque_orient.getZ();

    if(bDebug){
        for(std::size_t i = 0; i < F_msg_.data.size();i++){
            F_msg_.data[i] =F_ee_des_(i);
        }
        pub_F_.publish(F_msg_);
    }

    tau_cmd.data = J.data.transpose() * F_ee_des_;

    if(bDebug){
        for(std::size_t i = 0; i < tau_msg_.data.size();i++)
        {
            tau_msg_.data[i] = tau_cmd.data[i];
        }
        torque_pub_.publish(tau_msg_);
    }

    // tau_cmd.data.setZero();
}

void Passive_ds::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS);
    }
    bFirst            = true;
}
void Passive_ds::command_orient(const geometry_msgs::Quaternion &msg){
    rot_des_ = KDL::Rotation::Quaternion(msg.x,msg.y,msg.z,msg.w);
}

void Passive_ds::command_damping_eig(const std_msgs::Float64MultiArray& msg){

    if(msg.data.size() == 2){
        if(passive_ds_controller != NULL){
            passive_ds_controller->set_damping_eigval(msg.data[0],msg.data[1]);
            lwr_controllers::passive_ds_paramConfig config;
            dynamic_server_ds_param->getConfigDefault(config);

            config_cfg.damping_eigval0 = msg.data[0];
            config_cfg.damping_eigval1 = msg.data[1];
            dynamic_server_ds_param->updateConfig(config_cfg);

        }else{
            ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]  passive_ds_controller == NULL");
        }
    }else{
        ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]   msg.data.size() is not equal to 2, it is : " << msg.data.size());
    }
}

void Passive_ds::command_rot_stiff(const std_msgs::Float64& msg){
    rot_stiffness = msg.data;

    config_cfg.rot_stiffness = rot_stiffness;
    dynamic_server_ds_param->updateConfig(config_cfg);
}

void Passive_ds::command_rot_damp(const std_msgs::Float64& msg){
    rot_damping  = msg.data;

    config_cfg.rot_damping = rot_damping;
    dynamic_server_ds_param->updateConfig(config_cfg);
}

}
