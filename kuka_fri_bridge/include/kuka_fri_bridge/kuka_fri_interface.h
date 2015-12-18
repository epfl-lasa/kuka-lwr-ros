#ifndef KUKA_FRI_INTERFACE_H_
#define KUKA_FRI_INTERFACE_H_

#include "FastResearchInterface.h"
#include "LinuxAbstraction.h"
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include "kuka_fri_bridge/FRI.h"
#include "kuka_fri_bridge/LWRRobot_FRI.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"


namespace kfb {

class Fri_interface{

public:

    Fri_interface(ros::NodeHandle& nh);

    void start_fri();

    void publish(const kfb::LWRRobot_FRI& kuka_robot);

    boost::shared_ptr<FastResearchInterface>  mFRI;

private:


    std::vector<double>           tempEJT;

    ros::Publisher          fri_pub;
    ros::Publisher          joint_state_pub;
    ros::Publisher          cart_force_torque_pub;
    ros::Publisher          joint_external_ft_pub;
    kuka_fri_bridge::FRI    fri_data;
    sensor_msgs::JointState joint_states_msg;

    std_msgs::Float64MultiArray ee_ft_msg;
    std_msgs::Float64MultiArray j_ext_ft_msg;

    std::string             fri_drivers_path;


};
}


#endif
