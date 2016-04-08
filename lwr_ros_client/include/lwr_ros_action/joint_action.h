#ifndef LWR_ROS_ACTION__JOINT_ACTION_H_
#define LWR_ROS_ACTION__JOINT_ACTION_H_

#include "lwr_ros_action/base_action.h"
#include "lwr_ros_interface/ros_joint.h"
#include "lwr_ros_interface/switch_controller.h"
#include <array>

namespace ac{

class Joint_action : public ros_controller_interface::Ros_joint, public Base_action {

public:
    enum class MESSAGE_TYPE {JOINT_POSITION,JOINT_STIFFNESS,JOINT_DAMPING};

public:

    Joint_action(ros::NodeHandle&   nh,const std::string& controller_name = "joint_controllers");

    virtual bool update();

    virtual bool stop();

    void set_joint_values(std::array<double,KUKA_NUM_JOINTS> values,MESSAGE_TYPE type);

public:

    double              loop_rate_hz;
    double              stop_threash;
    bool                debug_print;

private:

    bool                                        b_damping,b_stiff,b_joint, b_run;
    Eigen::VectorXd                             joint_target_pos;
    double                                      joint_dist_norm;
    ros_controller_interface::Switch_controller switch_controller;

};

}



#endif
