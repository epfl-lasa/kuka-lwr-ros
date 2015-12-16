#ifndef KUKA_ACTION_SERVER_DEFAULT_TOPICS_H_
#define KUKA_ACTION_SERVER_DEFAULT_TOPICS_H_

#include <string>

namespace asrv{

class topics {

public:

    static const std::string EE_KUKA_POSE_TOPIC;
    static const std::string EE_STATE_POSE_TOPIC;
    static const std::string EE_STATE_FT_TOPIC;
    static const std::string EE_CMD_POSE_TOPIC;
    static const std::string EE_CMD_FT_TOPIC ;
    static const std::string EE_CMD_VEL_TOPIC;
    static const std::string EE_CMD_STIFF_TOPIC ;
    static const std::string BASE_LINK;

    static const std::string J_SENSE_TOPIC;
    static const std::string J_CMD_TOPIC;

   /* static const std::string J_STATE_POSE_TOPIC;
    static const std::string J_IMP_STATE_TOPIC;
    static const std::string J_CMD_POSE_TOPIC;
    static const std::string J_IMP_CMD_TOPIC;
    static const std::string J_ACTION_TOPIC;*/
};

}

#endif
