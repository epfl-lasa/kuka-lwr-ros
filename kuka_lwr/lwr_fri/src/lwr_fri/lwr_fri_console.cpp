#include "lwr_fri/lwr_fri_console.h"
#include "lwr_fri/utilities.h"
#include <future>
#include <thread>
#include <chrono>

namespace kfb{

Console_Interface::Console_Interface(string name, Fri_console* fri_console)
    :Console::Command(name),fri_console(fri_console)
{
}

Fri_console* Console_Interface::GetInterface(){
    return fri_console;
}

int Console_Interface::Execute(string args){
    return fri_console->RespondToConsoleCommand(m_Name,Tokenize(args));
}

Fri_console::Fri_console(ros::NodeHandle &nh)
{

    service_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/lwr/controller_manager/switch_controller");

    fri_sub = nh.subscribe("/lwr/FRI_data",1,&Fri_console::fri_callback,this);


    if(!ros::param::get("/start_controller",start_controller))
    {
        mConsole.Print("No start controller parameter set [Fri_console::Fri_console]!\nYou should set it via the fri_bridge.launch file.");
        std::this_thread::sleep_for (std::chrono::seconds(5));
        ROS_ERROR("No start controller parameter set [Fri_console::Fri_console]!\nYou should set it via the fri_bridge.launch file.");
        exit(0);
    }

    position.resize(NB_JOINTS);
    effort.resize(NB_JOINTS);
    damping.resize(NB_JOINTS);
    stiffness.resize(NB_JOINTS);

    IsRobotArmPowerOn          = false;
    DoesAnyDriveSignalAnError  = false;
}

void Fri_console::fri_callback(const lwr_fri::FRI::ConstPtr& msg){

    if(msg->position.size() == position.size()){
        position = msg->position;
    }
    if(msg->effort.size() == effort.size()){
        effort = msg->effort;
    }
    if(msg->damping.size() == damping.size()){
        damping = msg->damping;
    }
    if(msg->stiffness.size() == stiffness.size()){
        stiffness = msg->stiffness;
    }

    mFRI_STATE                  = int2FRI_STATE(msg->FRI_STATE);
    mFRI_QUALITY                = int2FRI_QUALITY(msg->FRI_QUALITY);
    mFRI_CTRL                   = int2FRI_CTRL(msg->FRI_CTRL);
    mROBOT_CTRL_MODE            = int2CTRL_STRATEGY(msg->ROBOT_CTRL_MODE);

    IsRobotArmPowerOn           = msg->IsRobotArmPowerOn;
    DoesAnyDriveSignalAnError   = msg->DoesAnyDriveSignalAnError;
}

int Fri_console::start(){
    mStdout = cout.rdbuf();
    cout.rdbuf(mOutputStream.rdbuf());
    mConsole.SetName("FRI");

    mConsole.AddCommand(new Console_Interface("stop",this));
    mConsole.AddCommand(new Console_Interface("control",this));

    mNCConsole.SetConsole(&mConsole);
    mNCConsole.InitNCurses();
    mNCConsole.SetTopStaticLinesCount(10);

    return 1;
}

int Fri_console::stop()
{
    usleep(100000);
    mNCConsole.FreeNCurses();
    std::cout<< "console stopped" << std::endl;
    return 1;
}

void Fri_console::addConsole(Console& console){
    mConsole.AddConsole(&console);
}

void Fri_console::ConsoleUpdate(){
    std::string s    = mOutputStream.str();
    std::size_t cpos = 0;
    std::size_t opos = 0;

    if(s.size()>0){
        for(unsigned int i=0;i<s.size();i++){
            opos = cpos;
            cpos = s.find("\n",opos);
            string ss = s.substr(opos,cpos-opos);
            if(ss.size()>0){
                mNCConsole.Print(ss);
            }
            if(cpos==string::npos)
                break;
            cpos++;
        }
        mOutputStream.str("");
    }
    int index = 0;
    if(position.size() == NB_JOINTS){
        sprintf(static_txt, "Joint Position  :  %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf", DEG(position[0]),DEG(position[1]),DEG(position[2]),DEG(position[3]),DEG(position[4]),DEG(position[5]),DEG(position[6]) );
        mNCConsole.SetTopStaticLine(index,  static_txt);
    }
    index++;

    if(effort.size() == NB_JOINTS){
        sprintf(static_txt, "Joint Torques   :  %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf   %-2.6lf", effort[0],effort[1],effort[2],effort[3],effort[4],effort[5],effort[6] );
        mNCConsole.SetTopStaticLine(index,  static_txt);
    }
    index++;

    if(stiffness.size() == NB_JOINTS){
        sprintf(static_txt, "Joint Stifness  :  %2.1lf       %2.1lf       %2.1lf       %2.1lf       %2.1lf       %2.1lf       %2.1lf",stiffness[0],stiffness[1],stiffness[2],stiffness[3],stiffness[4],stiffness[5],stiffness[6] );
        mNCConsole.SetTopStaticLine(index,  static_txt);
    }
    index++;

    if(damping.size() == NB_JOINTS){
        sprintf(static_txt, "Joint Damping   :  %2.2lf       %2.2lf       %2.2lf       %2.2lf       %2.2lf       %2.2lf       %2.2lf", damping[0],damping[1],damping[2],damping[3],damping[4],damping[5],damping[6] );
        mNCConsole.SetTopStaticLine(index,  static_txt);
    }
    index++;
    switch (mFRI_STATE) {
    case FRI_STATE_CMD:
        sprintf(static_txt, "FRI State       :  COMMAND");
        break;
    case FRI_STATE_MON:
        sprintf(static_txt, "FRI State       :  MONITOR");
        break;
    case FRI_STATE_INVALID:
        sprintf(static_txt, "FRI State       :  INVALID");
        break;
    case FRI_STATE_OFF:
        sprintf(static_txt, "FRI State       :  OFF");
        break;
    default:
        sprintf(static_txt, "FRI State       :  UNKNOWN");
        break;
    }

    mNCConsole.SetTopStaticLine(index,  static_txt);
    index++;

    switch (mFRI_QUALITY) {
    case FRI_QUALITY_BAD:
        sprintf(static_txt, "Comm. Quality   :  BAD");
        break;
    case FRI_QUALITY_INVALID:
        sprintf(static_txt, "Comm. Quality   :  INVALID");
        break;
    case FRI_QUALITY_OK:
        sprintf(static_txt, "Comm. Quality   :  OK");
        break;
    case FRI_QUALITY_PERFECT:
        sprintf(static_txt, "Comm. Quality   :  PERFECT");
        break;
    case FRI_QUALITY_UNACCEPTABLE:
        sprintf(static_txt, "Comm. Quality   :  UNACCEPTABLE");
        break;
    default:
        sprintf(static_txt, "Comm. Quality   :  UNKNOWN");
        break;
    }

    mNCConsole.SetTopStaticLine(index, static_txt);
    index++;
    //  typedef enum ControlStrategy {NONE=0, JOINT_POSITION = 10, CARTESIAN_IMPEDANCE = 20, JOINT_IMPEDANCE = 30, JOINT_EFFORT = 40, JOINT_STIFFNESS = 50, GRAVITY_COMPENSATION = 90} ControlStrategy;
    switch (mROBOT_CTRL_MODE) {
    case lwr_hw::LWRHW::ControlStrategy::NONE:
        sprintf(static_txt, "Control Mode    :  NONE");
        break;
    case lwr_hw::LWRHW::ControlStrategy::JOINT_POSITION:
        sprintf(static_txt, "Control Mode    :  JOINT POSITION");
        break;
    case lwr_hw::LWRHW::ControlStrategy::CARTESIAN_IMPEDANCE:
        sprintf(static_txt, "Control Mode    :  CARTESIAN_IMPEDANCE");
        break;
    case lwr_hw::LWRHW::ControlStrategy::JOINT_IMPEDANCE:
        sprintf(static_txt, "Control Mode    :  JOINT_IMPEDANCE");
        break;
    /*case lwr_hw::LWRHW::ControlStrategy::JOINT_EFFORT:
        sprintf(static_txt, "Control Mode    :  JOINT_EFFORT");
        break;*/
    default:
        sprintf(static_txt, "Control Mode    :  UNKNOWN");
        break;
    }

    mNCConsole.SetTopStaticLine(index, static_txt);
    index++;

    switch (mFRI_CTRL) {
    case FRI_CTRL_POSITION:
        sprintf(static_txt, "FRI CTRL Mode   :  JOINT POSITION");
        break;
    case FRI_CTRL_JNT_IMP:
        sprintf(static_txt, "FRI CTRL Mode   :  JOINT IMPEDANCE");
        break;
    case FRI_CTRL_CART_IMP:
        sprintf(static_txt, "FRI CTRL Mode   :  CARTESIAN IMPDEANCE");
        break;
    default:
        sprintf(static_txt, "FRI CTRL Mode   :  UNKNOWN");
        break;
    }

    mNCConsole.SetTopStaticLine(index, static_txt);
    index++;

    if(IsRobotArmPowerOn && !DoesAnyDriveSignalAnError)
        sprintf(static_txt, "Drives          :  GO");
    else
        sprintf(static_txt, "Drives          :  NO-GO");

    mNCConsole.SetTopStaticLine(index, static_txt);
    mNCConsole.Process();
    mNCConsole.Render();
}

int Fri_console::RespondToConsoleCommand(const string command, const vector<string> & args)
{

    mConsole.ClearLine();
    std::cout<< "respondToCondoleCommand" << std::endl;
    std::cout<< "command: " << command << std::endl;

    if(command == "stop")
    {
        mConsole.Print("stopping...");
        stop();
    }else if(command == "control"){
        switch_msg.request.start_controllers = {{start_controller}};
        switch_msg.request.stop_controllers.resize(0);
        switch_msg.request.strictness        = controller_manager_msgs::SwitchController::Request::STRICT;
        std::thread(&Fri_console::call_service_async,this).detach();
    }else{
        mConsole.Print("no such command: [" + command + "] supported!");
    }

    return true;
}

void Fri_console::call_service_async(){
    service_client.call(switch_msg);
}

}
