#ifndef CONTROLLERS_JOINT_POSITION_H_
#define CONTROLLERS_JOINT_POSITION_H_

#include <ros/ros.h>
#include "base_controllers.h"
#include "std_msgs/Float64MultiArray.h"
#include <boost/scoped_ptr.hpp>
#include <robot_motion_generation/CDDynamics.h>
#include <kdl/jntarrayacc.hpp>
#include "change_ctrl_mode.h"


namespace controllers{

class Joint_position : public Base_controllers {

public:

    Joint_position(ros::NodeHandle& nh,controllers::Change_ctrl_mode& change_ctrl_mode,const std::size_t num_joints=7);

    void update(KDL::JntArrayAcc& joint_des_, const KDL::JntArrayAcc &joint_msr_, const ros::Duration& period);

    void stop();

private:

    void command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg);

private:

    controllers::Change_ctrl_mode&                      change_ctrl_mode;
    ros::Subscriber                                     sub_command_joint_pos_;
    boost::scoped_ptr<motion::CDDynamics>               joint_cddynamics;
    KDL::JntArray                                       q_target_;
    std::size_t                                         num_joints;
    bool                                                bFirst,bFirst2;


};

}

#endif
