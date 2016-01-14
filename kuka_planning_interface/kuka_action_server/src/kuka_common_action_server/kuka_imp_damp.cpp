#include "kuka_common_action_server/kuka_imp_damp.h"

namespace asrv{

Kuka_imp_damp::Kuka_imp_damp(ros::NodeHandle&  nh):
Base_j_action(nh),
Base_ee_action(nh),
Base_action_server(nh)
{

}

bool Kuka_imp_damp::execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal){

    std::vector<double> stiffness = goal->JointStates.stiffness;
    std::vector<double> damping   = goal->JointStates.damping;

    std::string msg;
    if(stiffness.size() == 7){
        msg = "";
        for(std::size_t i = 0; i < 7;i++){
                msg = msg + boost::lexical_cast<std::string>(stiffness[i]) + " ";
        }
        ROS_INFO("stiffness values: %s",msg.c_str());
    }else{
        ROS_INFO("NO Stiffness values to be set!");
    }
    if(damping.size() != 0){
        msg = "";
        for(std::size_t i = 0; i < 7;i++){
                msg = msg + boost::lexical_cast<std::string>(damping[i]) + " ";
        }
        ROS_INFO("damping values: %s",msg.c_str());

    }else{
        ROS_INFO("NO Damping values to be set!");
    }

    std::string type = get_current_controller_name();
    ROS_INFO("ctrl: %s",type.c_str());

    if(type == "one_task_inverse_kinematics"){
        ROS_INFO("one_task_inverse_kinematics is active");
        if(stiffness.size() == ee_stiff_msg.data.size()){
            for(std::size_t i = 0; i < stiffness.size();i++){
                ee_stiff_msg.data[i] = stiffness[i];
            }
            Base_ee_action::sendStiff(ee_stiff_msg);
            ROS_INFO("setting ee stiffness");
        }
        if(damping.size() == ee_damp_msg.data.size()){
            for(std::size_t i = 0; i < damping.size();i++){
                ee_damp_msg.data[i] = damping[i];
            }
            Base_ee_action::sendDamp(ee_damp_msg);
            ROS_INFO("setting ee damping");
        }
    }else if(type == "joint_position_impedance_controller"){
        ROS_INFO("joint_position_impedance_controller is active");
        ROS_INFO("stiffness.size(): %d",stiffness.size());
        ROS_INFO("joint_stiff_msg.data.size(): %d",joint_stiff_msg.data.size());

        if(stiffness.size() == joint_stiff_msg.data.size()){
            for(std::size_t i = 0; i < stiffness.size();i++){
                joint_stiff_msg.data[i] = stiffness[i];
            }
            Base_j_action::sendStiff(joint_stiff_msg);
            ROS_INFO("setting j stiffness");
        }
        if(damping.size() == ee_damp_msg.data.size()){
            for(std::size_t i = 0; i < damping.size();i++){
                ee_damp_msg.data[i] = damping[i];
            }
            ROS_INFO("setting j damping");
            Base_ee_action::sendDamp(ee_damp_msg);
        }

    }else{
        ROS_ERROR("no such controller [%s] running",type.c_str());
        return false;
    }


    ROS_INFO("Finished set_imp_damp");
    return true;
}

}
