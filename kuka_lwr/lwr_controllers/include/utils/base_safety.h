#ifndef LWR_CONTROLLERS_BASE_SAFETY_H_
#define LWR_CONTROLLERS_BASE_SAFETY_H_

namespace lwr{
namespace safety{

class Base_safety{

public:

    virtual void reset()    = 0;
    virtual bool is_safe()  = 0;

};

}
}


#endif
