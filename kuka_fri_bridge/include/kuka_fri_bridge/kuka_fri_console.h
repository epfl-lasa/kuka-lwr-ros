#ifndef KUKA_FRI_BRIDGE_H_
#define KUKA_FRI_BRIDGE_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Empty.h"

#include "kuka_fri_bridge/utilities.h"

#include "std_tools/Console.h"
#include "std_tools/NCConsole.h"
#include "std_tools/Various.h"

#include "FastResearchInterface.h"
#include "LinuxAbstraction.h"

#include "boost/thread.hpp"
#include "time.h"
#include <deque>
#include <ros/package.h>

#include "kuka_fri_bridge/LWRRobot_FRI.h"
#include <controller_manager/controller_manager.h>
#include "kuka_fri_bridge/FRI.h"

#include <armadillo>

#define MEASUREMENT_HISTORY_LENGTH 20

#define MODE_NONE 0
#define MODE_REC  1
#define MODE_PREP 2
#define MODE_GO   3

#ifndef RAD
#define RAD(A)	((A) * M_PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / M_PI )
#endif


namespace kfb {

class Fri_console;

class Console_Interface : public Console::Command {

public:

    Console_Interface(string name, Fri_console *fri_console);

    Fri_console*    GetInterface();

    virtual int Execute(string args);

protected:

    Fri_console  *fri_console;

};


class Fri_console{

public:

       Fri_console(ros::NodeHandle& nh);

       int start();
       int stop();

       int RespondToConsoleCommand(const string cmd, const vector<string> &args);
       void addConsole(Console& console);
       void ConsoleUpdate();

private:

    void fri_callback(const kuka_fri_bridge::FRI::ConstPtr& msg);

private:

    ros::ServiceClient                          service_client;
    ros::Subscriber                             fri_sub;
    controller_manager_msgs::SwitchController   switch_msg;

    FRI_STATE                                   mFRI_STATE;
    FRI_QUALITY                                 mFRI_QUALITY;
    FRI_CTRL                                    mFRI_CTRL;

    std::vector<double>                         position, effort, damping, stiffness;

    bool    IsRobotArmPowerOn, DoesAnyDriveSignalAnError;


    NCConsole               mNCConsole;
    Console                 mConsole;

    streambuf *mStdout;
    stringstream mOutputStream;
    char static_txt[1025];

};


}


#endif
