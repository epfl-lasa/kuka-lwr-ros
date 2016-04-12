#include "utils/safety.h"

namespace lwr{
namespace safety{

Safety::Safety(ros::NodeHandle &nh,const std::string& topic_sub_name){

    safety_sub = nh.subscribe(topic_sub_name,2,&Safety::rest_safety_callback,this);
    safety_methods.clear();
    bIsSafe    = true;
}

void Safety::add(Base_safety* base_safe){
    safety_methods.push_back(base_safe);
}

bool Safety::is_safe(){
    for(std::size_t i = 0; i < safety_methods.size();i++){
        if(!safety_methods[i]->is_safe()){
            bIsSafe = false;
        }
    }

    return bIsSafe;
}

void Safety::reset(){
    ROS_WARN("Safety reset");
    for(std::size_t i = 0; i < safety_methods.size();i++){
        safety_methods[i]->reset();
    }
    bIsSafe=true;
}

void Safety::rest_safety_callback(const std_msgs::BoolConstPtr& msg){
    if(msg->data){
        reset();
    }
}

}
}
