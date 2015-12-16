#ifndef KUKA_ACTION_SERVER_ACTION_INITIALISER_H_
#define KUKA_ACTION_SERVER_ACTION_INITIALISER_H_

#include "kuka_action_server/default_topics.h"

#include <string>


namespace asrv {

class Base_action_initialiser{

public:

    Base_action_initialiser(){
        action_name             = "my_action";
        world_frame             = "/world_frame";
        model_dt                = 0.001; // [s]
        base_path               = "base_path";
    }

    std::string action_name;
    std::string world_frame;

    double      model_dt;
    std::string base_path;


};

class Action_ee_initialiser : public Base_action_initialiser{

public:

  Action_ee_initialiser():Base_action_initialiser()
    {
        reachingThreshold       = 0.01;  // [m]
        orientationThreshold    = 0.05;  // [rad]

        ee_state_pos_topic      = topics::EE_STATE_POSE_TOPIC;
        ee_cmd_pos_topic        = topics::EE_CMD_POSE_TOPIC;
        ee_cmd_vel_topic        = topics::EE_CMD_VEL_TOPIC;
        ee_cmd_ft_topic         = topics::EE_STATE_FT_TOPIC;
    }



    double      reachingThreshold;
    double      orientationThreshold;

    std::string ee_state_pos_topic;
    std::string ee_cmd_pos_topic;
    std::string ee_cmd_ft_topic;
    std::string ee_cmd_vel_topic;


};

class Action_j_initialiser : public Base_action_initialiser{

public:

    Action_j_initialiser():Base_action_initialiser(){
       /* j_state_pose_topic  = topics::J_STATE_POSE_TOPIC;
        j_cmd_pos_topic     = topics::J_CMD_POSE_TOPIC;
        j_imp_topic         = topics::J_IMP_STATE_TOPIC;
        j_imp_cmd_topic     = topics::J_IMP_CMD_TOPIC;
        j_action_topic      = topics::J_ACTION_TOPIC;*/
        j_sense_topic       = topics::J_SENSE_TOPIC;
        j_cmd_topic         = topics::J_CMD_TOPIC;
    }


    std::string j_sense_topic;
    std::string j_cmd_topic;

  /*  std::string j_state_pose_topic;
    std::string j_cmd_pos_topic;
    std::string j_imp_topic;
    std::string j_imp_cmd_topic;
    std::string j_action_topic;*/
};

}


#endif
