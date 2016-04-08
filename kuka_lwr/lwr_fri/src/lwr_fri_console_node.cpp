#include <ros/ros.h>
#include "lwr_fri/lwr_fri_interface.h"
#include "lwr_fri/lwr_fri_console.h"


int main(int argc, char** argv)
{

    ros::init(argc, argv, "LWR_FRI_CONSOLE", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("lwr_fri_console");

    kfb::Fri_console fri_console(nh);
    fri_console.start();

    ros::Rate r(50);
    while(ros::ok())
    {

        fri_console.ConsoleUpdate();
        ros::spinOnce();
        r.sleep();
    }

    fri_console.stop();

    return 0;
}
