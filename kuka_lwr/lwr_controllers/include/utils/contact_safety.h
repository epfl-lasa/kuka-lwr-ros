#ifndef LWR_CONTACT_SAFETY_H_
#define LWR_CONTACT_SAFETY_H_

/**
 *      Contact safety mechanism for when you are running the real kuka robot.
 *      If for instance the kuka enters in contact with the environment, depending
 *      on the strength of the contact the kuka should be set immediatly to gravity
 *      compensation.
 */

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include "utils/base_safety.h"

namespace lwr {
namespace safety{

class Contact_safety : public Base_safety{

public:

    Contact_safety(ros::NodeHandle& nh,const std::string& ee_ft_topic="ee_ft",const std::string& j_ext_ft_topic="j_ext_ft");

    void reset();

    bool is_safe();

private:

    void joint_external_force_callback(const std_msgs::Float64MultiArrayConstPtr& msg);

    void end_effector_external_force_callback(const geometry_msgs::WrenchStamped& msg);

private:

    ros::Subscriber     joint_external_ft_sub;
    ros::Subscriber     end_effector_external_ft_sub;

    double              max_ee_f;   // maximum magnitude of force on the end-effector allowed
    double              max_j_f;    // maximum magnitude of force on a joint allowed
    bool                bIsSafe;



};

}
}

#endif
