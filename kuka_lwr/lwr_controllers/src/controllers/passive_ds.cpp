#include "controllers/passive_ds.h"
#include <tf/transform_broadcaster.h>
#include <control_toolbox/filters.h>

namespace controllers {

static const double thrott_time = 3.0;

Passive_ds::Passive_ds(ros::NodeHandle &nh, controllers::Change_ctrl_mode &change_ctrl_mode):
    Base_controllers(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS),
    change_ctrl_mode(change_ctrl_mode)
{

    /// ROS topic

    sub_command_vel_       = nh.subscribe("passive_ds_command_vel",      1, &Passive_ds::command_cart_vel,     this , ros::TransportHints().reliable().tcpNoDelay());
    sub_command_nullspace_ = nh.subscribe("passive_ds_command_nullspace",  1 , &Passive_ds::command_nullspace,       this , ros::TransportHints().reliable().tcpNoDelay());
    sub_linear_damp_       = nh.subscribe("passive_ds_linear_damping",     1 , &Passive_ds::command_linear_damping_,  this , ros::TransportHints().reliable().tcpNoDelay());

    pub_twist_             = nh.advertise<geometry_msgs::Twist>("twist", 1);
    pub_F_                 = nh.advertise<std_msgs::Float64MultiArray>("F_ee", 10);
    torque_pub_            = nh.advertise<std_msgs::Float64MultiArray>("tau_pds", 10);

    /// Passive dynamical system
    nd5 = ros::NodeHandle(nh.getNamespace() + "/ds_param");

    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));


    dynamic_server_ds_param->setCallback(    boost::bind(&Passive_ds::ds_param_callback,     this, _1, _2));
    dynamic_server_ds_param->getConfigDefault(config_cfg);  // using the defaul values
    dynamic_server_ds_param->updateConfig(config_cfg);      // display the default values on the server (GUI)
    ds_param_callback(config_cfg, ~0);   // calling the callback to update the class variables


    dx_linear_des_.resize(3);
    dx_linear_msr_.resize(3);
    dx_angular_msr_.resize(3);


    F_ee_des_.resize(3);

    tau_msg_.data.resize(7);

    nullspace_torque.setConstant(0.0f);

    bFirst = false;


    _jointLimits << 170.0f, 120.0f, 170.0f, 120.0f, 170.0f, 120.0f, 170.0f;
    _jointLimits *= M_PI / 180.0f;

    qd <<     0.7055363655090332, 1.0911303758621216, -0.7358396053314209, -0.9389236569404602,
    0.5496764779090881, 0.8971629738807678, -1.3657774925231934;



}


void Passive_ds::stop() {
    ROS_INFO_STREAM("stopping [PASSIVE_DS]");
    bFirst      = false;
}

void Passive_ds::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config, uint32_t level) {
    // passive_ds_controller->set_damping_eigval(config.damping_eigval0, config.damping_eigval1);

    damping_x_ = config.damping_x;
    damping_y_ = config.damping_y;
    damping_z_ = config.damping_z;

    bDebug        = config.debug;
    bSmooth       = config.bSmooth;
    smooth_val_   = config.smooth_val;
    _useNullSpace = config.useNullSpace;
    _desiredJointsGain = config.desiredJointsGain;
    _jointVelocitiesGain = config.jointVelocitiesGain;

    config_cfg    = config;
}


void Passive_ds::update(KDL::Wrench &wrench, KDL::JntArray& tau_cmd, const KDL::Jacobian &J, const KDL::JntArrayAcc& joint_msr_ , const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_, const KDL::Vector& p) {


    F_ee_des_.setZero();

    /// set desired linear velocity
    dx_linear_des_(0)   = x_des_vel_(0);
    dx_linear_des_(1)   = x_des_vel_(1);
    dx_linear_des_(2)   = x_des_vel_(2);

    /// set measured linear and angular velocity
    if (bSmooth)
    {
        dx_linear_msr_(0)   = x_msr_vel_.vel(0);
        dx_linear_msr_(1)   = x_msr_vel_.vel(1);
        dx_linear_msr_(2)   = x_msr_vel_.vel(2);

        // dx_angular_msr_(0)  = x_msr_vel_.rot(0);
        // dx_angular_msr_(1)  = x_msr_vel_.rot(1);
        // dx_angular_msr_(2)  = x_msr_vel_.rot(2);
    } else {
        dx_linear_msr_(0)    = filters::exponentialSmoothing(x_msr_vel_.vel(0), dx_linear_msr_(0), smooth_val_);
        dx_linear_msr_(1)    = filters::exponentialSmoothing(x_msr_vel_.vel(1), dx_linear_msr_(1), smooth_val_);
        dx_linear_msr_(2)    = filters::exponentialSmoothing(x_msr_vel_.vel(2), dx_linear_msr_(2), smooth_val_);

        // dx_angular_msr_(0)   = filters::exponentialSmoothing(x_msr_vel_.rot(0), dx_angular_msr_(0), smooth_val_);
        // dx_angular_msr_(1)   = filters::exponentialSmoothing(x_msr_vel_.rot(1), dx_angular_msr_(1), smooth_val_);
        // dx_angular_msr_(2)   = filters::exponentialSmoothing(x_msr_vel_.rot(2), dx_angular_msr_(2), smooth_val_);
    }


    // ----------------- Linear velocity -> Force -----------------------//

    // passive_ds_controller->Update(dx_linear_msr_, dx_linear_des_);
    // _damping = (passive_ds_controller->damping_matrix()).cast<float>();

    // the damping controller:
    F_ee_des_(0) = - damping_x_ * (dx_linear_msr_(0) - dx_linear_des_(0));
    F_ee_des_(1) = - damping_y_ * (dx_linear_msr_(1) - dx_linear_des_(1));
    F_ee_des_(2) = - damping_z_ * (dx_linear_msr_(2) - dx_linear_des_(2));


    // ----------------- Debug -----------------------//

    // ROS_WARN_STREAM_THROTTLE(1, "velocity :" << dx_linear_des_ );


    // if (bDebug) {

    //     std::string robot_name = nd5.getNamespace().substr(0, nd5.getNamespace().find("/", 1));
    //     {
    //         static tf::TransformBroadcaster br;
    //         tf::Transform transform;
    //         transform.setOrigin( tf::Vector3(p(0), p(1), p(2)) );
    //         rot_msr_.GetQuaternion(qx, qy, qz, qw);
    //         q = tf::Quaternion(qx, qy, qz, qw);
    //         transform.setRotation(q);
    //         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name + "/rot_msr_"));
    //     }
    //     {
    //         static tf::TransformBroadcaster br;
    //         tf::Transform transform;
    //         transform.setOrigin( tf::Vector3(p(0), p(1), p(2)) );
    //         rot_des_.GetQuaternion(qx, qy, qz, qw);
    //         q = tf::Quaternion(qx, qy, qz, qw);
    //         transform.setRotation(q);
    //         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name + "/rot_target_"));
    //     }
    // }


    // damp any rotational motion






    tau_cmd.data = J.data.block(0, 0, 3, 7).transpose() * F_ee_des_;


    if (_useNullSpace)
    {

        // computing the torques
        Eigen::MatrixXd J_transpose_pinv;
        pseudo_inverse(J.data.block(0, 0, 3, 7).transpose(), J_transpose_pinv);
        nullspace_torque << (Eigen::MatrixXd::Identity(7, 7) - J.data.block(0, 0, 3, 7).transpose()*J_transpose_pinv)*
                         (-_desiredJointsGain * (joint_msr_.q.data - qd) - _jointVelocitiesGain * joint_msr_.qdot.data );


        if (bDebug)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "Nullspace command :" << qd);
            ROS_WARN_STREAM_THROTTLE(1.0, "Nullspace torques:" << nullspace_torque );
        }

        tau_cmd.data +=  nullspace_torque;
    }



    if (bDebug) {
        ROS_WARN_STREAM_THROTTLE(1.0, "Forces :" << F_ee_des_ );
        for (std::size_t i = 0; i < tau_msg_.data.size(); i++)
        {
            tau_msg_.data[i] = tau_cmd.data[i];
        }
        torque_pub_.publish(tau_msg_);
    }

    // seems like "wrench" is a useless variable, only for printing
    wrench.force(0) = F_ee_des_(0);
    wrench.force(1) = F_ee_des_(1);
    wrench.force(2) = F_ee_des_(2);
    wrench.torque(0) = 0;
    wrench.torque(1) = 0;
    wrench.torque(2) = 0;

    geometry_msgs::Twist msg;
    msg.linear.x = dx_linear_msr_(0);
    msg.linear.y = dx_linear_msr_(1);
    msg.linear.z = dx_linear_msr_(2);
    msg.angular.x = dx_angular_msr_(0);
    msg.angular.y = dx_angular_msr_(1);
    msg.angular.z = dx_angular_msr_(2);
    pub_twist_.publish(msg);

}

void Passive_ds::command_cart_vel(const geometry_msgs::TwistConstPtr &msg) {
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    if (!bFirst) {
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS);
    }
    bFirst            = true;
}





void Passive_ds::command_linear_damping_(const std_msgs::Float64MultiArray& msg) {

    if (msg.data.size() == 3) {

        if (msg.data[0] >= 0 && msg.data[1] >= 0 && msg.data[2] >= 0 ) {

            config_cfg.damping_x = msg.data[0];
            config_cfg.damping_y = msg.data[1];
            config_cfg.damping_z = msg.data[2];
            dynamic_server_ds_param->updateConfig(config_cfg);
            ds_param_callback(config_cfg, ~0);   // forcing the callback
        } else {
            ROS_ERROR_STREAM_THROTTLE(thrott_time, "[Passive_ds]  damping gains cannot be negative ");
        }


    } else {
        ROS_ERROR_STREAM_THROTTLE(thrott_time, "[Passive_ds::damping gains]   msg.data.size() is not equal to 3 (x-y-z), it is : " << msg.data.size());
    }

}



void Passive_ds::command_nullspace(const std_msgs::Float32MultiArray& msg) {

    if (msg.data.size() == 7)  {
        for (int k = 0 ; k < 7; k++)  {
            if (msg.data[k] > _jointLimits[k])
            {
                qd(k) = _jointLimits[k];
                ROS_ERROR_STREAM_THROTTLE(thrott_time, "[Passive_ds::nullspace command]   desired joint position out of bound for joint "
                                          <<  k << "  the limit is " << _jointLimits[k]);


            } else if (msg.data[k] < -_jointLimits[k]) {
                qd(k) = - _jointLimits[k];
                ROS_ERROR_STREAM_THROTTLE(thrott_time, "[Passive_ds::nullspace command]   desired joint position out of bound for joint "
                                          <<  k << "  the limit is " << -_jointLimits[k]);

            } else {
                qd(k) = msg.data[k];
            }
        }
    }
}

}

