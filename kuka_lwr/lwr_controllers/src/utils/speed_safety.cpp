#include "utils/speed_safety.h"

namespace lwr {
namespace safety{

const double warning_throttle_secs = 0.5;

Speed_safety::Speed_safety(ros::NodeHandle &nh,const KDL::FrameVel &ee_vel,const KDL::JntArray &qdot_msr_):
ee_vel(ee_vel),
qdot_msr_(qdot_msr_)
{

    max_ee_dt  = 0.1; // m/s
    max_qdot   = 0.1; // rad/s
    max_qddot  = 0.1; // rad/s2

    if (!nh.getParam("max_ee_dt",max_ee_dt) ){
        ROS_WARN_STREAM("max end-effector velocity not found, set to default " << max_ee_dt <<  " m/s");
    }

    if (!nh.getParam("max_qdot",max_qdot) ){
        ROS_WARN_STREAM("max joint velocity not found, set to default " << max_qdot <<  " m/s");
    }

    if (!nh.getParam("max_qddot",max_qddot) ){
        ROS_WARN_STREAM("max joint acceleration not found, set to default " << max_qddot <<  " m/s2");
    }

    bIsSafe=true;
}

void Speed_safety::reset(){
    bIsSafe=true;
}

bool Speed_safety::is_safe(){

    // check end-effector velocity
    if(ee_vel.GetTwist().vel.Norm() >= max_ee_dt){
        ROS_WARN_STREAM_THROTTLE(warning_throttle_secs, "end-effector velocity is ["
                                 << ee_vel.GetTwist().vel.Norm() << "] max allowed is ["
                                 << max_ee_dt << "]");
        bIsSafe = false;
    }

    // check joint velocity
    for(std::size_t i = 0; i < qdot_msr_.rows();i++){
        if(qdot_msr_(i) >= max_qdot){
            ROS_WARN_STREAM_THROTTLE(warning_throttle_secs, "joint["<< i <<
                                     "]: velocity: [" << qdot_msr_(i)
                                     << "] rad/s >= " << max_qdot);
            bIsSafe = false;
        }
    }


    return bIsSafe;
}


}
}
