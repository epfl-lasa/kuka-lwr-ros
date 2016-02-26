#include "controllers/gravity_compensation.h"

namespace controllers{

Gravity_compensation::Gravity_compensation(ros::NodeHandle &nh,
                                           controllers::Change_ctrl_mode& change_ctrl_mode)
    :Base_controllers(lwr_controllers::CTRL_MODE::GRAV_COMP),change_ctrl_mode(change_ctrl_mode)
{

    sub_command_grav_      = nh.subscribe("command_grav",     1, &Gravity_compensation::command_grav,         this);
    bFirst                 = true;

}


void Gravity_compensation::stop(){
    ROS_INFO_STREAM("stopping [GRAVITY COMPENSATION]");
    bFirst                 = true;

}


void Gravity_compensation::update(KDL::JntArray& tau_cmd_,KDL::JntArray& pos_cmd,KDL::JntArray& K_cmd, KDL::JntArray& D_cmd,KDL::JntArrayAcc& joint_des_,const KDL::JntArrayAcc& joint_msr_){
    for(int i = 0; i < tau_cmd_.data.size();i++){
        joint_des_.q(i)     = joint_msr_.q(i);
        K_cmd(i)            = 0;
        D_cmd(i)            = 0;
        tau_cmd_(i)         = 0;
        pos_cmd(i)          = joint_msr_.q(i);
        joint_des_.qdot(i)  = 0;
    }

}

void Gravity_compensation::command_grav(const std_msgs::Bool& msg){
    if(msg.data && bFirst)
    {
        change_ctrl_mode.switch_mode(ctrl_mode);
        bFirst=false;
    }


   /* ROS_INFO_THROTTLE(1.0,"command_grav");
    if(msg.data == true)
    {
        ROS_INFO("Starting Gravity compensation");
        for(std::size_t i = 0; i < joint_handles_.size();i++){
            K_(i)               = 0;
            D_(i)               = 0;
            tau_cmd_(i)         = 0;
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
        }
        robot_ctrl_mode = ROBOT_CTRL_MODE::TORQUE_IMP;
        ctrl_mode       = CTRL_MODE::GRAV_COMP;
    }else{
        ROS_INFO("Stopping Gravity compensation");
        for(std::size_t i = 0; i < 7; i++){
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
            D_(i)               = D_tmp(i);
            K_(i)               = K_tmp(i);
        }
        ctrl_mode =  CTRL_MODE::JOINT_POSITION;
    }*/


}



}
