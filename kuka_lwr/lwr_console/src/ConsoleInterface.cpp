#include "lwr_console/ConsoleInterface.h"
#include "lwr_console/Various.h"



Console_interface::Console_interface(string name, ServiceConsoleInterface* service_console_interface)
    :Console::Command(name),service_console_interface(service_console_interface)
{
}

ServiceConsoleInterface* Console_interface::GetInterface(){
    return service_console_interface;
}

int Console_interface::Execute(string args){
    return service_console_interface->RespondToConsoleCommand(m_Name,Tokenize(args));
}


ServiceConsoleInterface::ServiceConsoleInterface(ros::NodeHandle &nh, const string &console_name)
    :nh(nh)
{

}

int ServiceConsoleInterface::start(){
    mStdout = cout.rdbuf();
    cout.rdbuf(mOutputStream.rdbuf());
    mConsole.SetName("Cmd");
    mNCConsole.SetConsole(&mConsole);
    mNCConsole.InitNCurses();
    mNCConsole.SetTopStaticLinesCount(2);
    return 1;
}

int ServiceConsoleInterface::stop(){
    usleep(100000);
    mNCConsole.FreeNCurses();
    std::cout<< "console stopped" << std::endl;
    return 1;
}

int ServiceConsoleInterface::RespondToConsoleCommand(const string cmd, const vector<string> &args){


    it = services.find(cmd);
    if(it != services.end())
    {
     //   (it->second).call();

    }else{
        mConsole.Print("No such command [" + cmd + "]");
    }


}


void ServiceConsoleInterface::ConsoleUpdate(){
    int index=0;
    sprintf(static_txt, "-----------------------\nKUKA PLANNING INTERFACE\n-----------------------\n");
    mNCConsole.SetTopStaticLine(index, static_txt);

    mNCConsole.Process();
    mNCConsole.Render();
}

