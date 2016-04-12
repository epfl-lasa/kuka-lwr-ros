#ifndef LWR_CONTROLLERS_SPEED_SAFETY_H_
#define LWR_CONTROLLERS_SPEED_SAFETY_H_

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/framevel.hpp>
#include "utils/base_safety.h"

namespace lwr {
namespace safety{

class Speed_safety : public Base_safety{

public:

    Speed_safety(ros::NodeHandle& nh, const KDL::FrameVel &ee_vel, const KDL::JntArray &qdot_msr_);

    void reset();

    bool is_safe();

private:

    bool  bIsSafe;

    const KDL::FrameVel &ee_vel;
    const KDL::JntArray &qdot_msr_;

    double max_ee_dt;   // max end-effector velocity
    double max_qdot;    // max joint velocity
    double max_qddot;   // max joint acceleration


};


}
}


#endif
