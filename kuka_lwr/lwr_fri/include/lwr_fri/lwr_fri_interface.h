#ifndef LWR_FRI_INTERFACE_H_
#define LWR_FRI_INTERFACE_H_

#include "FastResearchInterface.h"
#include "LinuxAbstraction.h"
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include "lwr_fri/FRI.h"
#include "lwr_fri/LWRRobot_FRI.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"


namespace kfb {

class Fri_interface{

public:

    Fri_interface(ros::NodeHandle& nh);

    void start_fri();

    void publish(const kfb::LWRRobot_FRI& kuka_robot);

    boost::shared_ptr<FastResearchInterface>  mFRI;

private:


    std::vector<double>           tempEJT;

    ros::Publisher              fri_pub;
    ros::Publisher              joint_state_pub;
    ros::Publisher              cart_force_torque_pub;
    ros::Publisher              joint_external_ft_pub;
    lwr_fri::FRI                fri_data;
    sensor_msgs::JointState     joint_states_msg;

    std_msgs::Float64MultiArray j_ext_ft_msg;
    geometry_msgs::WrenchStamped       ee_ft_msg;
    std::vector<double>     tmp;

    std::string             fri_drivers_path;



};
}


#endif
