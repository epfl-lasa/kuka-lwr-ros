#ifndef KUKA_PLANNING_INTERFACE___SIMPLE_ACTIONS_H_
#define KUKA_PLANNING_INTERFACE___SIMPLE_ACTIONS_H_

#include <ros/ros.h>
#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>
#include <kuka_common_action_server/action_initialiser.h>
#include <std_msgs/Float64MultiArray.h>


namespace simple_actions {


class Linear_cart_action : public asrv::Base_ee_action, public asrv::Base_action_server {

public:


    Linear_cart_action(ros::NodeHandle& nh);

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    void simple_line_policy(Eigen::Vector3d& linear_velocity,
                                  Eigen::Vector3d& angular_velocity,
                            const    tf::Vector3&  current_origin,
                            const    tf::Quaternion& current_orient,
                            double rate);

private:

    tf::Vector3     target_origin;
    tf::Vector3     target_p1, target_p2;
    tf::Vector3     first_origin;
    tf::Quaternion  target_orientation;
    bool bFirst;


};


}




#endif
