#ifndef KUKA_PLANNING_INTERFACE___SIMPLE_ACTIONS_H_
#define KUKA_PLANNING_INTERFACE___SIMPLE_ACTIONS_H_

#include <ros/ros.h>


#include "lwr_ros_action/base_action.h"
#include "lwr_ros_interface/switch_controller.h"
#include "lwr_ros_interface/ros_ee_j.h"

#include <std_msgs/Float64MultiArray.h>
#include <robot_motion_generation/CDDynamics.h>
#include <memory>

namespace simple_actions {


class Linear_cart_action : public ros_controller_interface::Ros_ee_j, public ac::Base_action {

public:


    Linear_cart_action(ros::NodeHandle& nh);

    bool update();

    bool stop();

private:

    void simple_line_policy(Eigen::Vector3d& linear_velocity,
                                  Eigen::Vector3d& angular_velocity,
                            const    tf::Vector3&  current_origin,
                            const    tf::Quaternion& current_orient,
                            double rate);


public:

    double              loop_rate_hz;
    bool                b_run;
    bool                b_position;


private:

    tf::Vector3     target_origin;
    tf::Vector3     target_p1, target_p2;
    tf::Vector3     first_origin;
    tf::Quaternion  target_R_p1, target_R_p2;
    tf::Quaternion  target_orientation;

    double          dist_target;

    std::unique_ptr<motion::CDDynamics> linear_cddynamics;
    std::unique_ptr<motion::CDDynamics> angular_cddynamics;

    ros_controller_interface::Switch_controller switch_controller;


    bool bSwitch;
    int target_id;
    int target_id_tmp;

    bool bFirst;


};


}




#endif
