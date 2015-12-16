#ifndef KUKA_ACTION_SERVER_DEFAULT_SERVICES_H_
#define KUKA_ACTION_SERVER_DEFAULT_SERVICES_H_

#include <ros/ros.h>
#include <state_transformers/String_cmd.h>

namespace asrv {

typedef enum CTRL_TYPE{
    POSITION,
    VELOCITY,
    JOINT,
    GRAV_COMP,
    NONE
}CTRL_TYPE;


/**
 * @brief set_control_type : sets the control type [position,velocity] on the state transformer
 * @param ctrl_type: [position,velocity]
 */
void inline set_control_type(CTRL_TYPE ctrl_type,ros::ServiceClient& state_transformer_service){
    state_transformers::String_cmd cmd;
    switch(ctrl_type)
    {
    case POSITION:
    {
        cmd.request.cmd = "ctrl position";
        break;
    }
    case VELOCITY:
    {
        cmd.request.cmd = "ctrl velocity";
        break;
    }
    case JOINT:
    {
        cmd.request.cmd = "ctrl joint";
        break;
    }
    case GRAV_COMP:
    {
        cmd.request.cmd = "ctrl grav";
        break;
    }
    case NONE:
    {
        cmd.request.cmd = "ctrl none";
        break;
    }
    default:
    {
        cmd.request.cmd = "ctrl none";
        break;
    }
    }


    state_transformer_service.call(cmd);
}

}

#endif
