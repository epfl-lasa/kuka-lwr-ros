#ifndef BASE_CONTROLLERS_H_
#define BASE_CONTROLLERS_H_

#include <string>
#include "utils/definitions.h"

namespace controllers{

class Base_controllers{

public:

    Base_controllers(lwr_controllers::CTRL_MODE  ctrl_mode):ctrl_mode(ctrl_mode)
    {

    }

    virtual void stop() = 0;

    lwr_controllers::CTRL_MODE get_ctrl_mode() const{
        return ctrl_mode;
    }

protected:

   lwr_controllers::CTRL_MODE ctrl_mode;


};

}

#endif
