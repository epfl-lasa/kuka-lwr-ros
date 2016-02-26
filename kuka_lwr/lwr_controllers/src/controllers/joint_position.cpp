#include "controllers/joint_position.h"

namespace controllers{

Joint_position::Joint_position(ros::NodeHandle& nh, controllers::Change_ctrl_mode &change_ctrl_mode, const std::size_t num_joints):
    Base_controllers(lwr_controllers::CTRL_MODE::JOINT_POSITION),
    change_ctrl_mode(change_ctrl_mode),
    num_joints(num_joints)
{

    sub_command_joint_pos_ = nh.subscribe("command_joint_pos",1, &Joint_position::command_joint_pos,    this);
    q_target_.resize(num_joints);

    /// Filter

    joint_cddynamics.reset(new motion::CDDynamics(7,1e-6,1));
    motion::Vector velLimits(7);
    for(std::size_t i = 0; i < 7; i++){
        velLimits(i)  = 0.25; // x ms^-1
    }
    joint_cddynamics->SetVelocityLimits(velLimits);
    bFirst  = true;
    bFirst2 = true;

}

void Joint_position::stop(){
    ROS_INFO_STREAM("stopping [JOINT_POSITION]");
    bFirst  = true;
    bFirst2 = true;
}

void Joint_position::update(KDL::JntArrayAcc& joint_des_,const KDL::JntArrayAcc& joint_msr_, const ros::Duration& period)
{
    if(bFirst2){
        joint_cddynamics->SetState(joint_msr_.q.data);
        bFirst2=false;
    }

    joint_cddynamics->SetDt(period.toSec());
    joint_cddynamics->SetTarget(q_target_.data);
    joint_cddynamics->Update();
    joint_cddynamics->GetState(joint_des_.q.data);
}

void Joint_position::command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg){
    ROS_INFO("INSIDE JOINT POSITION CALLBACK");


    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", num_joints);
    }
    else if (msg->data.size() != num_joints) {
        ROS_INFO("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        ROS_INFO("JOINT POSITION CTRL SETTING TARGET");
        for (std::size_t j = 0; j < num_joints; ++j){
            q_target_(j)     = msg->data[j];
        }
        if(bFirst){

            change_ctrl_mode.switch_mode(ctrl_mode);
            bFirst=false;
            bFirst2=true;
        }
    }

}


}
