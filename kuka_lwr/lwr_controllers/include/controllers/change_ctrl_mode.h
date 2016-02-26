#ifndef CONTROLLERS_CHANGE_CTRL_MODE_H_
#define CONTROLLERS_CHANGE_CTRL_MODE_H_

#include "utils/definitions.h"
#include "controllers/base_controllers.h"
#include <map>

namespace controllers{

class Change_ctrl_mode{

public:

    Change_ctrl_mode();

    Change_ctrl_mode(lwr_controllers::CTRL_MODE ctrl_mode, lwr_controllers::ROBOT_CTRL_MODE robot_ctrl_mode);

    void add(controllers::Base_controllers* base_controllers);

    void switch_mode(lwr_controllers::CTRL_MODE des_ctrl_mode);

    void switching();

    lwr_controllers::CTRL_MODE get_ctrl_mode();

    lwr_controllers::ROBOT_CTRL_MODE get_robot_ctrl_mode();

    bool is_switching() const;

private:

    void reset_except_des_mode();

private:

    bool b_switching;

private:

    int count;

    lwr_controllers::CTRL_MODE                           current_ctrl_mode, desired_ctrl_mode;
    lwr_controllers::ROBOT_CTRL_MODE                     current_robot_ctrl_mode;
    std::map<lwr_controllers::CTRL_MODE,controllers::Base_controllers*> controllers_;
    std::map<lwr_controllers::CTRL_MODE,controllers::Base_controllers*>::iterator it;


};


}

#endif
