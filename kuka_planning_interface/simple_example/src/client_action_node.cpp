#include <ros/ros.h>

#include "kuka_action_client/action_client_cmd_interface.h"
#include "kuka_action_client/kuka_action_client.h"
#include "kuka_action_client/ros_param_parser.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv,"action_client");
    ros::NodeHandle nh("action_client");

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/speech_topic"]           = "";
    param_name_value[node_name + "/action_service_name"]    = "";
    param_name_value[node_name + "/cmd_service_name"]       = "";
    param_name_value[node_name + "/action_server_name"]     = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }

    std::string speech_topic          =  param_name_value[node_name + "/speech_topic"];
    std::string action_serivce_name   = param_name_value[node_name  + "/action_service_name"];
    std::string cmd_service_name      = param_name_value[node_name  + "/cmd_service_name"];
    std::string action_server_name    =  param_name_value[node_name + "/action_server_name"];


    /** ------------- Initialise Action Client & Set Action-Goals -------------

      The Pour_client is initialsed. A set of actions and goals are defined
      add added to the action clients container which is a map. The key of
      the map is the name of the action and the value is the Goal.

    **/

    ac::Kuka_action_client kuka_action_client(action_server_name);


    std::map<std::string,ac::Goal> goals;
    enum{KUKA_DOF = 7};
    std::array<double,KUKA_DOF> des_position;
    kuka_fri_bridge::JointStates jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);

    {
        ac::Goal goal;
        des_position  =  {{-1.02974,0.471239,0.401426,-1.76278,-1.0472,-0.802851,0.785398}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["go_front"]           = goal;
    }
    {
        ac::Goal goal;
        des_position  =  {{0.803,0.4995,0.0286,-1.986,0.9915,-1.1997,-0.5516}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["go_left"]            = goal;
    }
    {
        ac::Goal goal;
        des_position  =  {{0,0.785398,0.122173,-2.01099,-0.174533,0.261799,0}};

        for(std::size_t i = 0; i < KUKA_DOF;i++){
            jointStateImpedance.position[i]      = des_position[i];
        }

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["go_home"]            = goal;
    }
    {
        ac::Goal goal;
        goal.action_type        = "linear";
        goals["linear"]         = goal;

    }


    kuka_action_client.push_back(goals);



    /**  ------------- Initialise Control cmd  interface  -------------
     *  The control command interface is an interface to the action client.
     *  It provied a ros service and a voice command interface such to
     *  command the client server to send desired action requests to the action server.
     */
    ac::Action_client_cmd_interface action_cmd_interface(nh,kuka_action_client,action_serivce_name,cmd_service_name);
    action_cmd_interface.init_nl_subscriber(speech_topic);


    ROS_INFO("action CLIENT started!");

    action_cmd_interface.console.start();

    ros::Rate rate(50);
    while(ros::ok()){

        action_cmd_interface.console.ConsoleUpdate();

        rate.sleep();
        ros::spinOnce();
    }

    action_cmd_interface.console.stop();

    return 0;
}
