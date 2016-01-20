#ifndef ACTION_CLIENT_CMD_INTERFACE_H_
#define ACTION_CLIENT_CMD_INTERFACE_H_

#include <ros/ros.h>

#include "kuka_action_client/String_cmd.h"
#include "kuka_action_client/kuka_action_client.h"
#include "kuka_action_client/kuka_action_console.h"

#include <vector>
#include <functional>
#include <future>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

namespace ac{


typedef actionlib::SimpleClientGoalState::StateEnum action_states;

class Action_client_cmd_interface{

public:

    Action_client_cmd_interface(ros::NodeHandle          &nh,
                          ac::Kuka_action_client  &kuka_action_client,
                          const std::string        &action_service_name,
                          const std::string       &cmd_service_name);

    void init_nl_subscriber(std::string topic_name);

private:

    void nl_command_callback(const std_msgs::String::ConstPtr &msg);

private:

    bool action_service_callback(kuka_action_client::String_cmd::Request& req,kuka_action_client::String_cmd::Response &res);

    bool cmd_interface_callback(kuka_action_client::String_cmd::Request& req,kuka_action_client::String_cmd::Response &res);

    void action_cmd_callback(const std::string& action_cmd);

public:

    ac::Action_client_console          console;

private:

    ros::NodeHandle&                   nh;
    ac::Kuka_action_client&            kuka_action_client;
    actionlib::SimpleClientGoalState   curr_action_state;


    ros::Subscriber                    nl_sub_;
    boost::thread                      worker_thread;
    ros::ServiceServer                 action_service;
    ros::ServiceServer                 cmd_interface_service;
    std_msgs::String                   server_msg;
};

}

#endif
