#ifndef KUKA_PLANNING_INTERFACE___KUKA_ACTION_CONSOLE_H_
#define KUKA_PLANNING_INTERFACE___KUKA_ACTION_CONSOLE_H_

#include "std_tools/Console.h"
#include "std_tools/NCConsole.h"
#include "std_tools/Various.h"
#include <ros/ros.h>

#include "kuka_action_client/String_cmd.h"
#include "kuka_action_client/kuka_action_client.h"

#include <vector>
#include <functional>
#include <future>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

namespace ac{

class Action_client_console;

class Console_Interface : public Console::Command {

public:

    Console_Interface(string name, Action_client_console *action_client_console);

    Action_client_console*    GetInterface();

    virtual int Execute(string args);

protected:

    Action_client_console  *action_client_console;

};


class Action_client_console{
    typedef actionlib::SimpleClientGoalState::StateEnum action_states;

public:

       Action_client_console(ros::NodeHandle& nh, ac::Kuka_action_client  &kuka_action_client);

       int start();
       int stop();

       void AddConsoleCommand(const std::string& command);

       int RespondToConsoleCommand(const string cmd, const vector<string> &args);

       void addConsole(Console& console);
       void ConsoleUpdate();

private:


    NCConsole               mNCConsole;
    Console                 mConsole;

    ros::NodeHandle&                   nh;
    ac::Kuka_action_client&            kuka_action_client;
    std::vector<ros::Subscriber>       subs;
    boost::thread                      worker_thread;
    ros::ServiceServer                 action_service;
    ros::ServiceServer                 cmd_interface_service;


    actionlib::SimpleClientGoalState   curr_action_state;


    streambuf *mStdout;
    stringstream mOutputStream;
    char static_txt[1025];
};



}

#endif
