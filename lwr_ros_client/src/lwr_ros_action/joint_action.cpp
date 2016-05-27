#include "lwr_ros_action/joint_action.h"
#include "lwr_ros_interface/lwr_ros_utilities.h"

namespace ac{

Joint_action::Joint_action(ros::NodeHandle&   nh, const std::string& controller_name):
Ros_joint(nh,controller_name),
switch_controller(nh)
{
     loop_rate_hz       = 100;
     b_damping          = false;
     b_stiff            = false;
     b_joint            = false;
     b_run              = true;
     debug_print        = false;
     stop_threash       = 0.01;
     joint_dist_norm    = std::numeric_limits<double>::max();
     joint_target_pos.resize(KUKA_NUM_JOINTS);
}


bool Joint_action::update(){

        if(!switch_controller.activate_controller("joint_controllers"))
        {
            ROS_WARN_STREAM("failed to start controller [Joint_action::update()]!");
            return false;
        }

        if(b_damping){
            sendDamp(joint_damp_msg);
            ros::spinOnce();
        }

        if(b_stiff){
            sendDamp(joint_stiff_msg);
            ros::spinOnce();
        }

        if(!b_joint){
            ROS_WARN_STREAM("joint values not set [Joint_action::update()]!");
            return false;
        }

        // wait until first position message received

        sendJointPos(joint_target_pos);

        b_run = true;
        ros::Rate loop_rate(loop_rate_hz);
        while(b_run) {


            joint_dist_norm = (joint_target_pos - joint_sensed).norm();

            if(debug_print){
                ros_controller_interface::utilities::ros_print_joint(3.0,joint_sensed);
            }

            if(joint_dist_norm <= stop_threash)
            {
                b_run = false;
                ROS_INFO_STREAM("Joint action arrived at target [error: " << joint_dist_norm << "]");
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

        return true;
}

bool Joint_action::stop(){
    b_run = false;
    sendJointPos(joint_sensed);
}

void Joint_action::set_joint_values(std::array<double,7> values,MESSAGE_TYPE type){

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS; i++){

        if(type == MESSAGE_TYPE::JOINT_POSITION )
        {
            joint_cmd_msg.data[i] = values[i];
            joint_target_pos(i)   = values[i];
            b_joint               = true;
        }else if(type == MESSAGE_TYPE::JOINT_DAMPING){
            joint_damp_msg.data[i] = values[i];
            b_damping              = true;
        }else if(type == MESSAGE_TYPE::JOINT_STIFFNESS){
            joint_stiff_msg.data[i] = values[i];
            b_stiff                 = true;
        }

    }

}

}
