#ifndef   LWR_CONTROLLERS_DEFINITIONS_H_
#define  LWR_CONTROLLERS_DEFINITIONS_H_

#include <string>

namespace lwr_controllers{

enum class CTRL_MODE{
    CART_VELOCITIY,         /// velocity
    JOINT_POSITION,         /// standard joint position controller (for goto joint pose)
    GRAV_COMP               /// sets the controller into gravity compensation
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
    case CTRL_MODE::JOINT_POSITION:
        return "JOINT_POSITION";
    case CTRL_MODE::GRAV_COMP:
        return "GRAV_COMP";
    }

}

}


#endif
