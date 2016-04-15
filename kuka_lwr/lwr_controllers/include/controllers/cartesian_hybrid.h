#ifndef  CONTROLLERS_CARTESIAN_POSITION_H_
#define  CONTROLLERS_CARTESIAN_POSITION_H_

#include "utils/definitions.h"
#include "controllers/change_ctrl_mode.h"
#include "controllers/base_controllers.h"
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>

namespace controllers{

class Cartesian_hybrid : public Base_controllers {

public:

    Cartesian_hybrid(ros::NodeHandle &nh,controllers::Change_ctrl_mode& change_ctrl_mode);

    void stop();

private:

    void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);

private:

    Change_ctrl_mode&   change_ctrl_mode;

    ros::Subscriber     sub_command_pose_;
    KDL::Frame          x_des_;	//desired pose

    bool                cmd_flag_, bFirst;


};

}

#endif
