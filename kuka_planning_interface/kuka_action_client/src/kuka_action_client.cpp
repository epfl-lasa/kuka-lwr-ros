#include "kuka_action_client/kuka_action_client.h"
#include <array>

namespace ac{

Kuka_action_client::Kuka_action_client(const std::string& name)
    :ac_(name,true)
{
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer(); //will wait for infinite time
    //   joint_position_ac.wait_for_server();
    ROS_INFO("Action server started");

    // default initial state, no actions are running
    b_action_running    = false;
    current_action_name = "NONE";

    add_default_actions();
}

void Kuka_action_client::push_back(std::map<std::string,Goal>& goals){
    (this->goals).insert(goals.begin(), goals.end());
}

void Kuka_action_client::push_back(const Goal& goal,const std::string& name){
    goals[name] = goal;
}

bool Kuka_action_client::call_action(const std::string& name){

    std::map<std::string,Goal>::iterator it;

    it = goals.find(name);
    if(it != goals.end()){

        b_action_running        = true;
        current_action_name     = name;

        ac_.sendGoal(it->second);
        ac_.waitForResult();

        actionlib::SimpleClientGoalState state = ac_.getState();

        ROS_INFO("Action finished: %s",state.toString().c_str());
        b_action_running    = false;
        current_action_name = "NONE";
        return true;
    }else{
        std::string msg = "no such action defined: " + name;
        ROS_ERROR("%s",msg.c_str());
        return false;
    }
}

bool Kuka_action_client::has_action(const std::string& name){

    std::map<std::string,Goal>::iterator it;
    it = goals.find(name);
    if(it != goals.end()){
        return true;
    }else{
        return false;
    }
}

void Kuka_action_client::print_action_names() {
    std::map<std::string,Goal>::iterator it;
    it = goals.begin();
    std::cout<<std::endl;
    std::cout<< "=== action names ===" << std::endl;
    for(it = goals.begin(); it != goals.end();it++){
        std::cout<< " " << it->first << std::endl;
    }
    std::cout<<std::endl;
}

void Kuka_action_client::add_default_actions(){

    enum{KUKA_DOF = 7};

    std::array<double,KUKA_DOF> des_stiffness;

    kuka_fri_bridge::JointStates JointStates;
    JointStates.position.resize(KUKA_DOF);
    // JointStates.velocity.resize(KUKA_DOF);
    // JointStates.effort.resize(KUKA_DOF);
    JointStates.stiffness.resize(KUKA_DOF);

    ///--- Gravity Compensation Actions ---///
    // Go to Gravity Compensation
    {
        ac::Goal goal;
        des_stiffness =  {{0,0,0,0,0,0,0}};

        for(std::size_t i = 0; i < KUKA_DOF;i++){
            JointStates.stiffness[i]     = des_stiffness[i];
        }
        goal.action_type            = "grav_comp";
        goal.JointStates            = JointStates;
        goals["grav_comp"]          = goal;
    }

}


}
