#ifndef CONSOLE_INTERFACE_H_
#define CONSOLE_INTERFACE_H_

#include "lwr_console/Console.h"
#include "lwr_console/NCConsole.h"
#include <ros/ros.h>
#include <boost/thread.hpp>

class ServiceConsoleInterface;

class Console_interface : public Console::Command {

public:

    Console_interface(string name, ServiceConsoleInterface *service_console_interface);

    ServiceConsoleInterface*    GetInterface();

    virtual int Execute(string args);

protected:

    ServiceConsoleInterface  *service_console_interface;

};


class ServiceConsoleInterface{

public:

    ServiceConsoleInterface(ros::NodeHandle& nh, const std::string& console_name = "GIVE ME A NAME");

    int start();

    int stop();

    template <typename T>
    void AddConsoleCommand(const std::string& command, const std::string& service_name){
        services.insert(std::pair<std::string,ros::ServiceClient>(command,nh.serviceClient<T>(service_name)));
        mConsole.AddCommand(new Console_interface(command,this));
    }

    int  RespondToConsoleCommand(const string cmd, const vector<string> &args);

    void addConsole(Console& console);

    void ConsoleUpdate();

private:

    ros::NodeHandle&                                   nh;
    NCConsole                                          mNCConsole;
    Console                                            mConsole;
    std::map<std::string,ros::ServiceClient>           services;
    std::map<std::string,ros::ServiceClient>::iterator it;

    boost::thread                        worker_thread;
    streambuf                            *mStdout;
    stringstream                         mOutputStream;
    char static_txt[1025];

};



#endif
