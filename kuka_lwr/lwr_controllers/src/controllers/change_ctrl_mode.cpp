#include "controllers/change_ctrl_mode.h"
#include <ros/ros.h>

namespace controllers {

Change_ctrl_mode::Change_ctrl_mode(){
    current_ctrl_mode       =  lwr_controllers::CTRL_MODE::GRAV_COMP;
    current_robot_ctrl_mode = lwr_controllers::ROBOT_CTRL_MODE::TORQUE_IMP;
    b_switching             = false;
    count                   = 0;
}

Change_ctrl_mode::Change_ctrl_mode(lwr_controllers::CTRL_MODE ctrl_mode, lwr_controllers::ROBOT_CTRL_MODE robot_ctrl_mode)
{
    current_ctrl_mode           = ctrl_mode;
    current_robot_ctrl_mode     = robot_ctrl_mode;
    b_switching                 = false;
    count                       = 0;
}

bool Change_ctrl_mode::is_switching() const{
    return b_switching;
}

void Change_ctrl_mode::add(controllers::Base_controllers* base_controllers){
    controllers_[base_controllers->get_ctrl_mode()] = base_controllers;
}

void Change_ctrl_mode::switch_mode(lwr_controllers::CTRL_MODE des_ctrl_mode){
     std::cout<< "switch_mode #1" << std::endl;
     std::cout<< "b_switching: " << b_switching << std::endl;
    if(b_switching){
        ROS_WARN_STREAM("allready switching to, cannot switch to " << lwr_controllers::ctrl_mod2str(des_ctrl_mode));
        return;
    }

     std::cout<< "switch_mode #2" << std::endl;

    if(controllers_.find(des_ctrl_mode) == controllers_.end()){
        std::cout<< "Could not switch to [" << lwr_controllers::ctrl_mod2str(des_ctrl_mode) << "], it does not exist [Change_ctrl_mode::switch_mode]" << std::endl;
        ROS_WARN_STREAM("Could not switch to [" << lwr_controllers::ctrl_mod2str(des_ctrl_mode) << "], it does not exist [Change_ctrl_mode::switch_mode]");
        b_switching = false;
    }else{
        desired_ctrl_mode = des_ctrl_mode;
        b_switching       = true;
        count             = 0;
    }

    std::cout<< "b_switching: " << b_switching << std::endl;
}

void Change_ctrl_mode::switching(){
    if(count == 0){
        ROS_INFO_STREAM("SWITCHING [" << lwr_controllers::ctrl_mod2str(current_ctrl_mode) << "] ===> [" << lwr_controllers::ctrl_mod2str(desired_ctrl_mode) << "]");
        reset_except_des_mode();
        b_switching=true;
    }else if (count > 10)
    {
        b_switching       = false;
        current_ctrl_mode = desired_ctrl_mode;
        count             = 0;
        ROS_INFO_STREAM("finished SWITCHING current ctrl_mode [" << lwr_controllers::ctrl_mod2str(current_ctrl_mode) << "]");
    }
    count++;

}


void Change_ctrl_mode::reset_except_des_mode(){
    for(it = controllers_.begin(); it != controllers_.end(); it++) {
        if(it->first != desired_ctrl_mode){
            (it->second)->stop();
        }
    }
}

lwr_controllers::CTRL_MODE Change_ctrl_mode::get_ctrl_mode(){
    return current_ctrl_mode;
}

lwr_controllers::ROBOT_CTRL_MODE Change_ctrl_mode::get_robot_ctrl_mode(){
    return current_robot_ctrl_mode;
}

}
