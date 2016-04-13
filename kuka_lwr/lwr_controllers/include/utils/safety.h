#ifndef LWR_CONTROLLERS_SAFETY_H_
#define LWR_CONTROLLERS_SAFETY_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "utils/base_safety.h"
#include <vector>

namespace lwr{
namespace safety{

class Safety : public Base_safety{

public:

    Safety(ros::NodeHandle& nh,const std::string& topic_sub_name="lwr_safety");

    void add(Base_safety* base_safe);

    bool is_safe();

    void reset();

private:

    void rest_safety_callback(const std_msgs::BoolConstPtr& msg);

private:

    ros::Subscriber                     safety_sub;
    std::vector<Base_safety*>           safety_methods;
    bool                                bIsSafe;

};

}
}

#endif
