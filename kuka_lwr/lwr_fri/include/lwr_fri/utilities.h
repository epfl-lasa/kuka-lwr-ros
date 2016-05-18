#ifndef KUKA_FRI_BRIDGE_UTILITIES_H_
#define KUKA_FRI_BRIDGE_UTILITIES_H_

#include "fricomm.h"
#include "FastResearchInterface.h"
#include "lwr_hw/lwr_hw.h"


namespace kfb {

#define FRI_CONN_TIMEOUT_SEC	30

#define JOINT_MAX_VEL_DEG_SEC  60.0
#define JOINT_MAX_ACC_DEG_SEC  60.0
#define CART_MAX_VEL_M_SEC  0.2
#define CART_MAX_ACC_M_SEC  0.05

#define FRI_JOINT_STIFFNESS 500
#define FRI_JOINT_DAMPING   0.7

#define FRI_CART_STIFFNESS_POS 		300
#define FRI_CART_STIFFNESS_ORIENT 	30
#define FRI_CART_DAMPING_POS 		0.7
#define FRI_CART_DAMPING_ORIENT 	0.7

/*this parameter determines how often the impedance should be updated relative to the position.
 * example: if REL_FREQ_IMPEDANCE_UPDATE = 10, then the desired impedance will be sent to the robot
 * every 10 iterations of desrired pose command update.*/
#define REL_FREQ_IMPEDANCE_UPDATE 10

#define LWR_state_size 7+12+7+7+6+7+12
#define NB_JOINTS 7

/*
typedef enum ControlMode {
    CTRLMODE_NONE                   = 0,
    CTRLMODE_CARTIMPEDANCE          = 1,
    CTRLMODE_JOINTIMPEDANCE         = 2,
    CTRLMODE_GRAVITYCOMPENSATION    = 3,
    CTRLMODE_POSITION               = 4,
    CTRLMODE_TORQUE                 = 5,
} ControlMode;*/

/*typedef enum FRI_STATE{
    FRI_STATE_OFF=0,


} FRI_STATE;*/

inline FRI_QUALITY int2FRI_QUALITY(int i){
    if(i == 0){
        return FRI_QUALITY_UNACCEPTABLE;
    }else if(i == 1){
        return FRI_QUALITY_BAD;
    }else if(i == 2){
        return FRI_QUALITY_OK;
    }else if(i == 3){
        return FRI_QUALITY_PERFECT;
    }else if(i == -1){
        return FRI_QUALITY_INVALID;
    }else{
        return FRI_QUALITY_INVALID;
    }
}

inline FRI_STATE int2FRI_STATE(int i){
    if (i == 0){
        return FRI_STATE_OFF;
    }else if(i == 1){
        return FRI_STATE_MON;
    }else if(i == 2){
        return FRI_STATE_CMD;
    }else if(i == -1){
        return FRI_STATE_INVALID;
    }else{
        std::cout<< "no such FRI_STATE returning OFF" << std::endl;
        return FRI_STATE_INVALID;
    }
}
inline std::string fri_state2str(FRI_STATE fri_state){
    switch(fri_state)
    {
    case FRI_STATE_INVALID:
        return "FRI_STATE_INVALID";
    case FRI_STATE_OFF:
        return "FRI_STATE_OFF";
    case FRI_STATE_MON:
        return "FRI_STATE_MON";
    case FRI_STATE_CMD:
        return "FRI_STATE_CMD";
    }
}




inline FRI_CTRL FastFRI_CTRL2FRI_CTRL(int i){
    switch (i)
    {
    case FastResearchInterface::JOINT_POSITION_CONTROL:
        return(FRI_CTRL_POSITION );
        break;
    case FastResearchInterface::CART_IMPEDANCE_CONTROL:
        return(FRI_CTRL_CART_IMP);
        break;
    case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
        return(FRI_CTRL_JNT_IMP);
        break;
    case FastResearchInterface::NONE:
        return(FRI_CTRL_OTHER);
    default:
        return(FRI_CTRL_OTHER);
        break;
    }
}

inline FRI_CTRL int2FRI_CTRL(int i){
    if(i == 1){
        return FRI_CTRL_POSITION;
    }else if(i == 2){
        return FRI_CTRL_CART_IMP;
    }else if(i == 3){
        return FRI_CTRL_JNT_IMP;
    }else{
        return FRI_CTRL_OTHER;
    }
}

inline lwr_hw::LWRHW::ControlStrategy int2CTRL_STRATEGY(int i)
{
    switch(i)
    {
    case 0:
    {
        return lwr_hw::LWRHW::NONE;
    }
    case 10:
    {
        return lwr_hw::LWRHW::JOINT_POSITION;

    }
    case 20:
    {
        return lwr_hw::LWRHW::CARTESIAN_IMPEDANCE;
    }
    case 30:
    {
        return lwr_hw::LWRHW::JOINT_IMPEDANCE;

    }
    case 90:
    {
        return lwr_hw::LWRHW::GRAVITY_COMPENSATION;
    }
    default:
    {
        return lwr_hw::LWRHW::NONE;
    }
    }
}

}

#endif
