#ifndef  CONTROLLERS_GRAVITY_COMPENSATION_H_
#define  CONTROLLERS_GRAVITY_COMPENSATION_H_

#include "utils/definitions.h"
#include "controllers/base_controllers.h"
#include <map>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include "controllers/change_ctrl_mode.h"


namespace controllers{

class Gravity_compensation : public Base_controllers {

public:

    Gravity_compensation(ros::NodeHandle& nh,controllers::Change_ctrl_mode& change_ctrl_mode);

    void update(KDL::JntArray &tau_cmd_,KDL::JntArray& pos_cmd,KDL::JntArray& K, KDL::JntArray& D, KDL::JntArrayAcc &joint_des_, const KDL::JntArrayAcc &joint_msr_);

    void stop();

private:

    void command_grav(const std_msgs::Bool& msg);

private:

    controllers::Change_ctrl_mode& change_ctrl_mode;
    ros::Subscriber sub_command_grav_;
    bool            bFirst;

};


}


#endif
