#ifndef   LWR_CONTROLLERS_DEFINITIONS_H_
#define  LWR_CONTROLLERS_DEFINITIONS_H_

#include <string>

namespace lwr_controllers{

enum class CTRL_MODE{
    CART_VELOCITIY,         /// velocity
    CART_POSITION,          /// position
    CART_PASSIVE_DS,        /// passive ds (velocity cart)
    JOINT_POSITION,         /// standard joint position controller (for goto joint pose)
    GRAV_COMP,              /// sets the controller into gravity compensation
    FF_FB_CARTESIAN         /// feedforward + feedback trajectory for the end effector
};

enum class ROBOT_CTRL_MODE
{
    POSITION_IMP,
    TORQUE_IMP
};

inline std::string ctrl_mod2str(CTRL_MODE mode)
{
    switch (mode) {
    case CTRL_MODE::CART_VELOCITIY:
        return "CART_VELOCITIY";
    case CTRL_MODE::CART_PASSIVE_DS:
        return "CART_PASSIVE_DS";
    case CTRL_MODE::CART_POSITION:
        return "CART_POSITION";
    case CTRL_MODE::JOINT_POSITION:
        return "JOINT_POSITION";
    case CTRL_MODE::GRAV_COMP:
        return "GRAV_COMP";
    case CTRL_MODE::FF_FB_CARTESIAN:
        return "FF_FB_CARTESIAN";
    }

}

}


#endif
