#ifndef KUKA_ACTION_SERVER_KUKA_GRAV_AS_H_
#define KUKA_ACTION_SERVER_KUKA_GRAV_AS_H_

#include "kuka_action_server/default_types.h"
#include "kuka_action_server/base_action_server.h"

#include "kuka_action_server/base_j_action.h"
#include "kuka_action_server/base_ee_action.h"
#include "kuka_common_action_server/action_initialiser.h"

#include "kuka_fri_bridge/FRI.h"


/**
  *     Gravity compendation
  *     For joint position impedance mode on the KUKA. When this action
  *     is called the stiffness values are set to zero for all joints,
  *     simulating gravitiy compensation. When the action is interupted
  *     the stiffness values will be reset to the previous ones.
  *
  */

namespace asrv{

class Kuka_grav_as:  public Base_j_action, public Base_action_server {

public:

    Kuka_grav_as(ros::NodeHandle&  nh);

    virtual bool execute_CB(alib_server& as_, alib_feedback& feedback_, const cptrGoal& goal);

private:

    void fri_callback(const kuka_fri_bridge::FRI::ConstPtr& msg);

private:

    ros::Subscriber         fri_sub;
    Eigen::VectorXd         stiffness,first_stiffness;

    // Desired Joint Impedance Command
    Eigen::VectorXd  des_j_stiffness;

};

}

#endif
