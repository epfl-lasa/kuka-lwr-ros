#include "controllers/cartesian_position.h"
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

namespace controllers{


Cartesian_position::Cartesian_position(ros::NodeHandle &nh,controllers::Change_ctrl_mode& change_ctrl_mode)
    :Base_controllers(lwr_controllers::CTRL_MODE::CART_POSITION),change_ctrl_mode(change_ctrl_mode)
{

    sub_command_pose_      = nh.subscribe("command_pos",      1, &Cartesian_position::command_cart_pos,     this);

    Kp = 2.0;
    Ki = 0.0;
    Kd = 0.0;

    cmd_flag_ = true;

}


void Cartesian_position::update(const KDL::Frame& x_,const KDL::Jacobian& J_,KDL::JntArrayAcc& joint_des, const ros::Duration& period){

    if(cmd_flag_){

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // end-effector position error
        //KDL::Frame tmp;
        //tmp.p = P_err.p;

        //P_err.p = x_des_.p - x_.p;
        //I_err.p = P_err.p + I_err.p;
        //D_err.p = P_err.p - tmp.p;
        // x_err_.vel = Kp * P_err.p + Ki*I_err.p + Kd * D_err.p;
         x_err_.vel = x_des_.p - x_.p;


        // getting quaternion from rotation matrix
        x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
        x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

        skew_symmetric(quat_des_.v, skew_);

        for (int i = 0; i < skew_.rows(); i++)
        {
            v_temp_(i) = 0.0;
            for (int k = 0; k < skew_.cols(); k++)
                v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
        }

        // end-effector orientation error
        x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

        // computing q_dot
        for (int i = 0; i < J_pinv_.rows(); i++)
        {
            joint_des.qdot(i) = 0.0;
            for (int k = 0; k < J_pinv_.cols(); k++)
                joint_des.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
        }

        ROS_INFO_STREAM_THROTTLE(0.5,"cart error: " << x_err_.vel(0) << " " << x_err_.vel(1) << " " << x_err_.vel(2));

        if( Equal(x_, x_des_, 0.005) ){
            ROS_INFO("On target");
            cmd_flag_ = false;
        }

        // integrating q_dot -> getting q (Euler method)
        for (std::size_t  i = 0; i < static_cast<std::size_t>(joint_des.q.data.size()); i++){
            joint_des.q(i) += period.toSec()*joint_des.qdot(i);
        }
    }
}

void Cartesian_position::stop(){
    ROS_INFO_STREAM("stopping [CARTESIAN POSITION]");
    bFirst      = false;
    cmd_flag_   = false;
}

void Cartesian_position::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{

   // ROS_INFO("================================================> command_cart_pos callback!");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));

    x_des_      = frame_des_;
    cmd_flag_   = true;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_POSITION);
    }
    bFirst            = true;

}



}
