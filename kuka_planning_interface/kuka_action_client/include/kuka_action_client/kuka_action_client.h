#ifndef KUKA_ACTION_CLIENT_H_
#define KUKA_ACTION_CLIENT_H_

/**

   KUKA Action client

   Handles all interaction with server client, all action requests to
   the action server and done via this class. Tipically add set set
   of actions with assicated goals to the action client and use the callback
   function to send these requests to the action server.

  **/

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include "kuka_action_server/PLAN2CTRLAction.h"
#include "kuka_action_client/common_action_client/joint_position_ac.h"


#include <map>

namespace ac{

typedef actionlib::SimpleActionClient<kuka_action_server::PLAN2CTRLAction>      alib_client;
typedef kuka_action_server::PLAN2CTRLGoal                                       Goal;
typedef actionlib::SimpleClientGoalState::StateEnum                             action_states;

class Kuka_action_client{

public:

    Kuka_action_client(const std::string &name);

    /**
     * @brief push_back : adds a set of goals to the goal container
     * @param goals     : map with key [action_name] and target [goal class].
     */
    void push_back(std::map<std::string,Goal>& goals);

    /**
     * @brief push_back : adds a goal to the list of goals available to the action client.
     * @param goal      : goal description.
     * @param name      : action name.
     */
    void push_back(const Goal& goal,const std::string& name);

    /**
     * @brief call_action : makes the action client send an action request [name]
     *                      to the action server.
     * @return            : returns the outcome of the execution of the action.
     */
    bool call_action(const std::string& name);

    bool has_action(const std::string& name);

    void print_action_names();



private:


    /**
     * @brief add_default_actions   : adds default action definitions, such as gravity compensation,
     *                                etc...
     */
    void add_default_actions();


public:

    alib_client                ac_;

    /**
     * @brief goals :  list of action names with associated goal descriptions.
     */
    std::map<std::string,Goal> goals;

    /**
     * @brief b_action_running : true if an action is currently running on the action
     *                           server and false otherwise.
     */
    volatile bool              b_action_running;
    volatile bool              action_finished;

    /**
     * @brief current_action_name : name of the action which is currently being run on the
     * action server. If no actions are currently in execution the value will be NONE.
     */
    std::string                current_action_name;



};

}

#endif
