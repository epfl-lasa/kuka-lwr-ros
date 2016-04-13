#include <thread>
#include <boost/thread.hpp>
#include "lwr_ros_client/action_client_cmd_interface.h"

namespace ac{

Action_client_cmd_interface::Action_client_cmd_interface(ros::NodeHandle    &nh,
                                             ac::Kuka_action_client         &kuka_action_client,
                                             const std::string              &action_service_name,
                                             const std::string              &cmd_service_name)
    :
    nh(nh),
    kuka_action_client(kuka_action_client)
{
    action_service             = nh.advertiseService(action_service_name,&Action_client_cmd_interface::action_service_callback,this);

    if(cmd_service_name != ""){
        cmd_interface_service      = nh.advertiseService(cmd_service_name,&Action_client_cmd_interface::cmd_interface_callback,this);
    }
}

void Action_client_cmd_interface::init_nl_subscriber(std::string topic_name){
    nl_sub_ = nh.subscribe(topic_name,1,&Action_client_cmd_interface::nl_command_callback,this);
}

void Action_client_cmd_interface::nl_command_callback(const std_msgs::String::ConstPtr& msg){
    std::string action_name = msg->data;
    ROS_INFO("I heard [%s]",action_name.c_str());
    action_cmd_callback(action_name);
}

bool Action_client_cmd_interface::action_service_callback(lwr_ros_client::String_cmd::Request& req,lwr_ros_client::String_cmd::Response &res){

    std::cout<< "Action_client_cmd_interface::service_callback" << std::endl;
    std::string action_name         = req.cmd;
    action_cmd_callback(action_name);
    res.res = "";
    return true;
}

void Action_client_cmd_interface::action_cmd_callback(const std::string& action_name){

    std::string current_action_name = kuka_action_client.current_action_name;

    std::cout<< "=== Service call back === " <<                                 std::endl;
    std::cout<< " current action:        "   << current_action_name          << std::endl;
    std::cout<< " requested action:      "   << action_name                  << std::endl;

   if(!kuka_action_client.b_action_running){
        std::cout<< " start action:          "   << action_name << std::endl;
        boost::thread( boost::bind( &ac::Kuka_action_client::call_action, boost::ref(kuka_action_client),action_name ) );
    }else{
       kuka_action_client.stop_action(current_action_name);
       kuka_action_client.b_action_running = false;
       worker_thread.join();
    }
}


bool Action_client_cmd_interface::cmd_interface_callback(lwr_ros_client::String_cmd::Request& req,lwr_ros_client::String_cmd::Response &res){

    std::string cmd = req.cmd;

    if(cmd == "help"){
        std::cout<< "no help option implemented yet" <<std::endl;
        res.res = "help";
        return true;
    }else if(cmd == "print actions"){
        kuka_action_client.print_action_names();
        res.res = "actions printed";
        return true;
    }else{
        res.res = "no such command defined [" + cmd  + "] in Action_client_cmd_interface::cmd_interface_callback";
        return false;
    }


}



}
