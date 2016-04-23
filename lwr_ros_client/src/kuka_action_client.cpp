#include "lwr_ros_client/kuka_action_client.h"
#include <array>

namespace ac{

Kuka_action_client::Kuka_action_client()
{
    current_action_name = "NONE";
    b_action_running    = false;
}

void Kuka_action_client::push_back(std::map<std::string, ac::Base_action*> &action_callback){
    (this->actions).insert(action_callback.begin(), action_callback.end());
}

void Kuka_action_client::push_back(const std::string& name, ac::Base_action* action){
    actions[name] = action;
}

bool Kuka_action_client::call_action(const std::string& name){

    ROS_INFO_STREAM("Kuka_action_client::call_action: " << name);
    std::map<std::string,ac::Base_action*>::iterator it;

    it = actions.find(name);
    if(it != actions.end()){

        ac::Base_action*        ptr_action;
        current_action_name     = name;
        b_action_running        = true;

        ptr_action = (it->second);
        ptr_action->update();

        b_action_running        = false;
        current_action_name     = "NONE";
        return true;
    }else{
        b_action_running        = false;
        std::string msg = "no such action defined: " + name;
        ROS_ERROR("%s",msg.c_str());
        return false;
    }
}

void Kuka_action_client::stop_action(const std::string& name){
    std::map<std::string,ac::Base_action*>::iterator it;
    it = actions.find(name);
    if(it != actions.end()){
        ROS_INFO_STREAM("stopping [" << name << "]");
        (it->second)->stop();
    }
}

bool Kuka_action_client::has_action(const std::string& name){

    std::map<std::string,ac::Base_action*>::iterator it;
    it = actions.find(name);
    if(it != actions.end()){
        return true;
    }else{
        return false;
    }
}

void Kuka_action_client::print_action_names() {
    std::map<std::string,ac::Base_action*>::iterator it;
    it = actions.begin();
    std::cout<<std::endl;
    std::cout<< "=== action names ===" << std::endl;
    for(it = actions.begin(); it != actions.end();it++){
        std::cout<< " " << it->first << std::endl;
    }
    std::cout<<std::endl;
}


}
