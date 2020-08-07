#ifndef LWR_ROS_ACTION__BASE_ACTION_H_
#define LWR_ROS_ACTION__BASE_ACTION_H_

namespace ac{

class Base_action{

public:

    virtual bool update() = 0;

    virtual bool stop() = 0;


};

}

#endif
