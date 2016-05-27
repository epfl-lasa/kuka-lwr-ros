#include "controllers/passive_ds.h"
#include <tf/transform_broadcaster.h>

namespace controllers{

static const double thrott_time = 3.0;

Passive_ds::Passive_ds(ros::NodeHandle &nh, controllers::Change_ctrl_mode &change_ctrl_mode):
 Base_controllers(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS),
  change_ctrl_mode(change_ctrl_mode)
{

    /// ROS topic

    sub_command_vel_       = nh.subscribe("passive_ds_command_vel",      1, &Passive_ds::command_cart_vel,     this);
    sub_command_orient_    = nh.subscribe("passive_ds_command_orient",   1 ,&Passive_ds::command_orient,       this);

    /// Passive dynamical system

    passive_ds_controller.reset(new DSController(3,50.0,50.0));

    nd5 = ros::NodeHandle("ds_param");
    nd6 = ros::NodeHandle("rot_stiffness");

    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));
    dynamic_server_rot_stiffness_param.reset(new   dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig>(nd6));


    dynamic_server_ds_param->setCallback(    boost::bind(&Passive_ds::ds_param_callback,     this, _1, _2));
    dynamic_server_rot_stiffness_param->setCallback( boost::bind(&Passive_ds::rot_stiffness_callback,this, _1, _2));

    for(std::size_t i = 0; i < 9; i++){
        err_orient.data[i] = 0;
    }

    dx_linear_des_.resize(3);
    dx_linear_msr_.resize(3);
    dx_angular_msr_.resize(3);
    F_ee_des_.resize(6);
    bFirst = false;

    rot_stiffness = 0;

    x_des_.M = KDL::Rotation::RPY(0,0,0);

    /// ROS pub debug

    pub_F_                 = nh.advertise<std_msgs::Float64MultiArray>("F_ee",10);
    torque_pub_            = nh.advertise<std_msgs::Float64MultiArray>("tau_pds",10);
    F_msg_.data.resize(6);
    tau_msg_.data.resize(7);
}


void Passive_ds::stop(){
    ROS_INFO_STREAM("stopping [PASSIVE_DS]");
    bFirst      = false;
}

void Passive_ds::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
}

void Passive_ds::rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level){
    rot_stiffness = config.rot_stiffness;
}


void Passive_ds::update(KDL::JntArray& tau_cmd, const KDL::Jacobian &J, const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_){



    /// set desired linear velocity
    dx_linear_des_(0)   = x_des_vel_(0);
    dx_linear_des_(1)   = x_des_vel_(1);
    dx_linear_des_(2)   = x_des_vel_(2);

    /// set measured linear and angular velocity
    dx_linear_msr_(0)   = x_msr_vel_.vel(0);
    dx_linear_msr_(1)   = x_msr_vel_.vel(1);
    dx_linear_msr_(2)   = x_msr_vel_.vel(2);

    dx_angular_msr_(0)  = x_msr_vel_.rot(0);
    dx_angular_msr_(1)  = x_msr_vel_.rot(1);
    dx_angular_msr_(2)  = x_msr_vel_.rot(2);

    // ----------------- Linear velocity -> Force -----------------------//

    passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)

    F_ee_des_(0) = F_linear_des_(0);
    F_ee_des_(1) = F_linear_des_(1);
    F_ee_des_(2) = F_linear_des_(2);

    // ----------------- Debug -----------------------//


   /* x_des_.p(0)         = p_(0) + x_des_vel_(0);
    x_des_.p(1)         = p_(1) + x_des_vel_(1);
    x_des_.p(2)         = p_(2) + x_des_vel_(2);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_des_.p(0), x_des_.p(1), x_des_.p(2)) );
    tf::Quaternion q;
    ROS_INFO_STREAM_THROTTLE(thrott_time,"x_q: " << x_des_orient_.getX() << " " << x_des_orient_.getY() << " " << x_des_orient_.getZ() << " " << x_des_orient_.getW());
    transform.setRotation(x_des_orient_);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_des_orient_"));*/

    // ----------------- Rotation target -> Force -----------------------//


    // damp any rotational motion
    //  x_des_orient_rot_.setRotation(x_des_orient_);
    err_orient = rot_msr_ * x_des_.M.Inverse();//x_des_orient_rot_.transpose();

    err_orient.GetQuaternion(qx,qy,qz,qw);
    tf::Quaternion q(qx,qy,qz,qw);

    //	err_orient_axis = err_orient.GetNormRotationAxis();
    err_orient_axis = q.getAxis();
    //	err_orient_angle = err_orient.GetRotationAngle();
    err_orient_angle = q.getAngle();

    if(std::isnan(err_orient_angle) || std::isinf(err_orient_angle)){
        ROS_WARN_STREAM_THROTTLE(thrott_time,"err_orient_angle: " << err_orient_angle );
        err_orient_angle = 0;
    }

    //	// rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    //control_torque += err_orient_axis*err_orient_angle*(-rot_stiffness_);
    torque_orient = err_orient_axis * err_orient_angle * (-rot_stiffness);

    F_ee_des_(3) = -0.05 * dx_angular_msr_(0) + torque_orient.getX();
    F_ee_des_(4) = -0.05 * dx_angular_msr_(1) + torque_orient.getY();
    F_ee_des_(5) = -0.05 * dx_angular_msr_(2) + torque_orient.getZ();

    for(std::size_t i = 0; i < F_msg_.data.size();i++){
        F_msg_.data[i] =F_ee_des_(i);
    }
    pub_F_.publish(F_msg_);


    // ROS_INFO_STREAM_THROTTLE(thrott_time,"err_orient: " << r_e << " " << p_e << " " << y_e);
    // ROS_INFO_STREAM_THROTTLE(thrott_time,"err_orient_angle: "<< err_orient_angle);

    //	// combine in one wrench vector
    //	Vector cartesian_control(6);
    //	cartesian_control.SetSubVector(0,control_force);
    //	cartesian_control.SetSubVector(3,control_torque);
    // ROS_INFO_STREAM_THROTTLE(thrott_time,"F_ee_des_: " << F_ee_des_(0) << " " << F_ee_des_(1) << " " << F_ee_des_(2) << " " << F_ee_des_(3) << " " << F_ee_des_(4)  << " " << F_ee_des_(5)  );
     tau_cmd.data = J.data.transpose() * F_ee_des_;

     for(std::size_t i = 0; i < tau_msg_.data.size();i++)
     {
         tau_msg_.data[i] = tau_cmd.data[i];
     }
     torque_pub_.publish(tau_msg_);

     //tau_cmd.data.setZero();
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
    double x, y, z, w;
    x = msg.x;
    y = msg.y;
    z = msg.z;
    w = msg.w;
    x_des_.M.GetQuaternion(x,y,z,w);
}

void Passive_ds::command_damping_eig(const std_msgs::Float64MultiArray& msg){

    if(msg.data.size() == 2){
        if(passive_ds_controller != NULL){
            passive_ds_controller->set_damping_eigval(msg.data[0],msg.data[1]);
          }else{
            ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]  passive_ds_controller == NULL");
        }
    }else{
        ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]   msg.data.size() is not equal to 2, it is : " << msg.data.size());
    }
}

void Passive_ds::command_rot_stiff(const std_msgs::Float64& msg){
    rot_stiffness = msg.data;
}

}
