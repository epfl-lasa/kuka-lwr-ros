#include <ros/ros.h>
#include "kuka_fri_bridge/kuka_fri_interface.h"
#include "kuka_fri_bridge/kuka_fri_console.h"


int main(int argc, char** argv)
{

    ros::init(argc, argv, "KUKA_FRI_CONSOLE", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("kuka_fri_console");

    kfb::Fri_console fri_console(nh);
    fri_console.start();

    ros::Rate r(60);
    while(ros::ok())
    {

        fri_console.ConsoleUpdate();
        ros::spinOnce();
        r.sleep();
    }

    fri_console.stop();

    return 0;
}
