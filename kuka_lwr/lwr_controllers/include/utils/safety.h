#ifndef LWR_CONTROLLERS_SAFETY_H_
#define LWR_CONTROLLERS_SAFETY_H_

#include <ros/ros.h>
#include <kdl/jntarray.hpp>

namespace lwr {
namespace safety{

/**
 * @brief check_position    :   Makes sure that robot does not reach a maximimum specified velocity. It
 *                              reads the current joint velocities (rad/s). If one joint is above this
 *                              velocity it is set to max allowed.
 * @param tau_cmd_          :
 * @param q_cmd_            :
 * @param qdot_msr_         :
 * @param threashold        :
 */



inline bool is_too_fast(const KDL::JntArray& qdot_msr_,const double threashold){
    bool bToFast=false;
    for(std::size_t i = 0; i < qdot_msr_.rows();i++){
        if(qdot_msr_(i) >= threashold){
            ROS_WARN_STREAM_THROTTLE(0.5,"joint["<<i<<"]: velocity: [" << qdot_msr_(i) << "] rad/s >= " << threashold);
            bToFast = true;
        }
    }
    return bToFast;
}


}
}


#endif
