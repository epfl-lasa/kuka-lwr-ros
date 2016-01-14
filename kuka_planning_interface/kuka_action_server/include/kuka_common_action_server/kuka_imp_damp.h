#ifndef KUKA_ACTION_SERVER_SET_IMPEDANCE_H_
#define KUKA_ACTION_SERVER_SET_IMPEDANCE_H_

#include "kuka_action_server/default_types.h"
#include "kuka_action_server/base_action_server.h"

#include "kuka_action_server/base_j_action.h"
#include "kuka_action_server/base_ee_action.h"

namespace asrv{

class Kuka_imp_damp:  public Base_j_action,public Base_ee_action, public Base_action_server {

public:

    Kuka_imp_damp(ros::NodeHandle&  nh);

    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal);

private:


};
}


#endif
