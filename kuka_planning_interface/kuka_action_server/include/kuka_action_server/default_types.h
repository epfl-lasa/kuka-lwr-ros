#ifndef KUKA_ACTION_SERVER_DEFAULT_TYPES_H_
#define KUKA_ACTION_SERVER_DEFAULT_TYPES_H_

#include "actionlib/server/simple_action_server.h"
#include "kuka_action_server/PLAN2CTRLAction.h"


namespace asrv{

typedef actionlib::SimpleActionServer<kuka_action_server::PLAN2CTRLAction>    alib_server;
typedef kuka_action_server::PLAN2CTRLFeedback                                 alib_feedback;
typedef kuka_action_server::PLAN2CTRLResult                                   alib_result;
typedef kuka_action_server::PLAN2CTRLGoalConstPtr                             cptrGoal;

}

#endif
