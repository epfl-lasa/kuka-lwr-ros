#ifndef KUKA_ACTION_SERVER_ACTION_SERVER_H_
#define KUKA_ACTION_SERVER_ACTION_SERVER_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "kuka_action_server/base_action_server.h"
#include "kuka_action_server/default_types.h"

#include "kuka_common_action_server/kuka_grav_as.h"
#include "kuka_common_action_server/kuka_goto_joint_as.h"
#include "kuka_common_action_server/kuka_goto_cart_as.h"

#include <memory>

#include <std_msgs/String.h>
#include <functional>
#include <map>


namespace asrv{


/**
 * @brief The Action_server class : Contains all off the users implementation of verious control
 * polices. When the action server receives a request from the action client to run a particular
 * action, the action server will execute the callback associated with the requested action.
 */

class Action_server{

public:

    Action_server(ros::NodeHandle& nh,std::string name);

    /**
     * @brief push_back             : add's an action implementation with associated reference name
     *                                to the action server.
     * @param base_action_server    : all action implementations inherite "Base_action_server" for which
     *                                the user will have provided his own control policy implementation.
     */
    void push_back(Base_action_server* base_action_server,std::string action_name);

private:

    void executeCB(const cptrGoal &goal);

    void subscriber_cb(const std_msgs::String::ConstPtr& msg);


private:

    /**
     * @brief add_default_actions   : initialiases and adds default actions which are very common to
     *                                all users. Actions such as gravity compensation and goto cartesian
     *                                and joint positions.
     */
    void add_default_actions(ros::NodeHandle& nh);

    void print_actions() const;

private:

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    alib_server                         as_;
    std::string                         action_name_;
    // create messages that are used to published feedback/result
    alib_feedback                       feedback_;
    alib_result                         result_;


    Base_action_server*                  base_action_server;

    /**
     * @brief actions : key is the name of an action and the value is a pointer
     * to a particular control policy.
     */
    std::map<std::string,Base_action_server*>           actions;
    std::map<std::string,Base_action_server*>::iterator actions_it;

    // Default actions made available to the user
    std::shared_ptr<asrv::Kuka_grav_as>                 ptr_kuka_grav_as;
    std::shared_ptr<asrv::Kuka_goto_joint_as>           ptr_kuka_goto_joint_as;
    std::shared_ptr<asrv::Kuka_goto_cart_as>            ptr_kuka_goto_cart_as;

};

}


#endif
