#include "controllers/passive_ds.h"

namespace controllers{

Passive_ds::Passive_ds(ros::NodeHandle &nh){

    /// Passive dynamical system

    passive_ds_controller.reset(new DSController(3,50.0,50.0));


    nd5 = ros::NodeHandle("ds_param");
    nd6 = ros::NodeHandle("rot_stiffness");


    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));
    dynamic_server_rot_stiffness_param.reset(new   dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig>(nd6));


    dynamic_server_ds_param->setCallback(    boost::bind(&Passive_ds::ds_param_callback,     this, _1, _2));
    dynamic_server_rot_stiffness_param->setCallback( boost::bind(&Passive_ds::rot_stiffness_callback,this, _1, _2));


    dx_linear_des_.resize(3);
    dx_linear_msr_.resize(3);
    dx_msr_.resize(6);
    F_ee_des_.resize(6);
}


void Passive_ds::stop(){

}

void Passive_ds::update(){

}

void Passive_ds::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
}

void Passive_ds::rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level){
    rot_stiffness = config.rot_stiffness;
}



/*

void JointControllers::passive_ds_update(){
    ROS_INFO_STREAM_THROTTLE(thrott_time,"passive_ds_update()");

    x_orient_[0][0] = x_.M(0,0);
    x_orient_[0][1] = x_.M(0,1);
    x_orient_[0][2] = x_.M(0,2);

    x_orient_[1][0] = x_.M(1,0);
    x_orient_[1][1] = x_.M(1,1);
    x_orient_[1][2] = x_.M(1,2);

    x_orient_[2][0] = x_.M(2,0);
    x_orient_[2][1] = x_.M(2,1);
    x_orient_[2][2] = x_.M(2,2);

    /// set desired linear velocity
    dx_linear_des_(0) = x_des_vel_(0);
    dx_linear_des_(1) = x_des_vel_(1);
    dx_linear_des_(2) = x_des_vel_(2);

    /// 3 linear & 3 angular velocities
    dx_msr_ = J_.data * joint_msr_.qdot.data;

    /// set measured linear velocity
    dx_linear_msr_(0) = dx_msr_(0);
    dx_linear_msr_(1) = dx_msr_(1);
    dx_linear_msr_(2) = dx_msr_(2);

    ROS_INFO_STREAM_THROTTLE(thrott_time,"dx_linear_msr_: " << dx_linear_msr_(0) << " " << dx_linear_msr_(1) << " " << dx_linear_msr_(2));
    ROS_INFO_STREAM_THROTTLE(thrott_time,"dx_linear_des_: " << dx_linear_des_(0) << " " << dx_linear_des_(1) << " " << dx_linear_des_(2));


    // DSController.Update();
    passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)

    F_ee_des_(0) = F_linear_des_(0);
    F_ee_des_(1) = F_linear_des_(1);
    F_ee_des_(2) = F_linear_des_(2);

    // damp any rotational motion
    x_des_orient_rot_.setRotation(x_des_orient_);
    err_orient = x_orient_ * x_des_orient_rot_.transpose();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_.p(0), x_.p(1), x_.p(2)) );
    tf::Quaternion q;
    ROS_INFO_STREAM_THROTTLE(thrott_time,"x_q: " << x_des_orient_.getX() << " " << x_des_orient_.getY() << " " << x_des_orient_.getZ() << " " << x_des_orient_.getW());
    transform.setRotation(x_des_orient_);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_des_orient_"));

    double err_orient_angle;
    err_orient.getRotation(q);

    double r_e,p_e,y_e;
    err_orient.getRPY(r_e,p_e,y_e);

    //	err_orient_axis = err_orient.GetNormRotationAxis();
    tf::Vector3 err_orient_axis = q.getAxis();
    //	err_orient_angle = err_orient.GetRotationAngle();
    err_orient_angle            = q.getAngle();
    if(std::isnan(err_orient_angle) || std::isinf(err_orient_angle)){
        ROS_WARN_STREAM_THROTTLE(thrott_time,"err_orient_angle: " << err_orient_angle );
        err_orient_angle = 0;
    }

    //	// rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    //control_torque += err_orient_axis*err_orient_angle*(-rot_stiffness_);
    tf::Vector3 torque_orient = err_orient_axis * err_orient_angle * (-rot_stiffness);

    F_ee_des_(3) = -0.05 * dx_msr_(3) + torque_orient.getX();
    F_ee_des_(4) = -0.05 * dx_msr_(4) + torque_orient.getY();
    F_ee_des_(5) = -0.05 * dx_msr_(5) + torque_orient.getZ();

    /*  for(std::size_t i = 0; i < F_msg.data.size();i++){
        F_msg.data[i] =F_ee_des_(i);
    }
    pub_F_.publish(F_msg);
*/
    // ROS_INFO_STREAM_THROTTLE(thrott_time,"err_orient: " << r_e << " " << p_e << " " << y_e);
    // ROS_INFO_STREAM_THROTTLE(thrott_time,"err_orient_angle: "<< err_orient_angle);

    //	// combine in one wrench vector
    //	Vector cartesian_control(6);
    //	cartesian_control.SetSubVector(0,control_force);
    //	cartesian_control.SetSubVector(3,control_torque);
   /* ROS_INFO_STREAM_THROTTLE(thrott_time,"F_ee_des_: " << F_ee_des_(0) << " " << F_ee_des_(1) << " " << F_ee_des_(2) << " " << F_ee_des_(3) << " " << F_ee_des_(4)  << " " << F_ee_des_(5)  );

    tau_cmd_.data = J_.data.transpose() * F_ee_des_;
}
*/

}
