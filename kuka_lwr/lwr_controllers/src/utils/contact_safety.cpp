#include "utils/contact_safety.h"

namespace lwr {
namespace safety{



Contact_safety::Contact_safety(ros::NodeHandle& nh,const std::string& ee_ft_topic,const std::string& j_ext_ft_topic){

    max_ee_f    =   4;
    max_j_f     =   4;

    if ( !nh.getParam("max_ee_f",max_ee_f) ){
        ROS_WARN_STREAM("max end-effector force not found, set to default " << max_ee_f <<  " N");
    }

    if ( !nh.getParam("max_j_f",max_j_f) ){
        ROS_WARN_STREAM("max joint force not found, set to default " << max_j_f <<  " N");
    }

    joint_external_ft_sub           = nh.subscribe(j_ext_ft_topic,10,&Contact_safety::joint_external_force_callback,this);
    end_effector_external_ft_sub    = nh.subscribe(ee_ft_topic,10,&Contact_safety::end_effector_external_force_callback,this);
    bIsSafe                         = true;

}

void Contact_safety::reset(){

}

bool Contact_safety::is_safe(){
    return bIsSafe;
}


void Contact_safety::joint_external_force_callback(const std_msgs::Float64MultiArrayConstPtr& msg){


}

void Contact_safety::end_effector_external_force_callback(const geometry_msgs::WrenchStamped& msg){

}


}
}
