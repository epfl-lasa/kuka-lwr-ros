#include "kuka_action_client/kuka_action_console.h"

namespace ac{



Console_Interface::Console_Interface(string name, Action_client_console* action_client_console)
    :Console::Command(name),action_client_console(action_client_console)
{
}

Action_client_console* Console_Interface::GetInterface(){
    return action_client_console;
}

int Console_Interface::Execute(string args){
    return action_client_console->RespondToConsoleCommand(m_Name,Tokenize(args));
}



Action_client_console::Action_client_console(ros::NodeHandle &nh,
                                             ac::Kuka_action_client  &kuka_action_client)
    :nh(nh),
      kuka_action_client(kuka_action_client),
      curr_action_state(action_states::PENDING)
{

}

void Action_client_console::AddConsoleCommand(const std::string& command){
    mConsole.AddCommand(new Console_Interface(command,this));
}

int Action_client_console::start(){
    mStdout = cout.rdbuf();
    cout.rdbuf(mOutputStream.rdbuf());
    mConsole.SetName("Cmd");
    mNCConsole.SetConsole(&mConsole);
    mNCConsole.InitNCurses();
    mNCConsole.SetTopStaticLinesCount(2);
    return 1;
}

int Action_client_console::stop(){
    usleep(100000);
    mNCConsole.FreeNCurses();
    std::cout<< "console stopped" << std::endl;
    return 1;
}

int Action_client_console::RespondToConsoleCommand(const string cmd, const vector<string> &args){

    // command is an action
    if(kuka_action_client.has_action(cmd))
    {
        std::string current_action_name = kuka_action_client.current_action_name;
        std::string action_name         = cmd;

        std::cout<< "=== Service call back === " <<                                 std::endl;
        std::cout<< " current action:        "   << current_action_name          << std::endl;
        std::cout<< " requested action:      "   << action_name                  << std::endl;

       if(!kuka_action_client.b_action_running){
           std::cout<< " start action:          "   << action_name << std::endl;
            boost::thread( boost::bind( &ac::Kuka_action_client::call_action, boost::ref(kuka_action_client),action_name ) );
        }else{
            kuka_action_client.ac_.cancelAllGoals();
            kuka_action_client.b_action_running = false;
            worker_thread.join();
        }
    }

    // command is a sub-action


    return 1;
}

void Action_client_console::addConsole(Console& console){
    mConsole.AddConsole(&console);
}

void Action_client_console::ConsoleUpdate(){
    std::string s    = mOutputStream.str();
    /*std::size_t cpos = 0;
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
    }*/

    int index=0;
    sprintf(static_txt, "-----------------------\nKUKA PLANNING INTERFACE\n-----------------------\n");
    mNCConsole.SetTopStaticLine(index, static_txt);

    mNCConsole.Process();
    mNCConsole.Render();
}





}
