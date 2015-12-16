#include "kuka_action_server/action_server.h"
#include "kuka_action_client/ros_param_parser.h"

#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include "kuka_common_action_server/kuka_grav_as.h"

/**
    Including your own action libraray
  **/
#include "simple_actions/linear_cart_action.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "kuka_server");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh("kuka_server");

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];

    /**  ------------- Initialise your action  ------------- **/

    simple_actions::Linear_cart_action linear_cart_action(nh);



    /**  ------------- Initialise Action Server ------------- **/

    asrv::Action_server action_server(nh,action_server_name);


    /**  ------------- Push back policies ------------- **/

    action_server.push_back(&linear_cart_action,"linear");


    ROS_INFO("action SERVER started");
    ros::spin();

	return 0;
}
