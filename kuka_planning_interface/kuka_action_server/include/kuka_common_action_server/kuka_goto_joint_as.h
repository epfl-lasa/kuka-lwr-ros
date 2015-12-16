#ifndef KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_
#define KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_

#include "kuka_action_server/default_types.h"
#include "kuka_action_server/base_action_server.h"

#include "kuka_action_server/base_j_action.h"
#include "kuka_action_server/base_ee_action.h"
#include "kuka_common_action_server/action_initialiser.h"
#include "std_msgs/Float64MultiArray.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <robot_motion_generation/CDDynamics.h>


namespace asrv{

class Kuka_goto_joint_as:  public Base_j_action, public Base_action_server {

public:

    Kuka_goto_joint_as(ros::NodeHandle&  nh);

    virtual bool execute_CB(alib_server& as_, alib_feedback& feedback_, const cptrGoal& goal);

private:

    Eigen::VectorXd     joint_states_target;
    Eigen::VectorXd     joint_states_des;
    Eigen::VectorXd     joint_states_diff;
    double              model_dt;
    motion::CDDynamics* cddynamics;




};

}

#endif
