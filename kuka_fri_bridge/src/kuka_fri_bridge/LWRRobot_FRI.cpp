#include "kuka_fri_bridge/LWRRobot_FRI.h"
#include "LinuxAbstraction.h"
namespace kfb {

LWRRobot_FRI::LWRRobot_FRI(boost::shared_ptr<FastResearchInterface>& mFRI) :
    mFRI(mFRI),
    lwr_hw::LWRHW()
{

    mDesired_control_strategy = NONE;
    setControlStrategy(NONE);
    mSwitched                 = false;

}

LWRRobot_FRI::~LWRRobot_FRI(){
}
bool LWRRobot_FRI::init(){


    for(std::size_t i = 0; i < joint_stiffness_.size();i++){
        joint_stiffness_[i]             = 500;
        joint_damping_[i]               = 0;
        joint_stiffness_command_[i]     = joint_stiffness_[i];
        joint_damping_command_[i]       = joint_damping_command_[i];
    }

    return true;
}

void LWRRobot_FRI::safe_joint_target_update(ros::Time time, ros::Duration period){

}

void LWRRobot_FRI::read(ros::Time time, ros::Duration period)
{

    float msrJntPos[n_joints_];
    float msrJntTrq[n_joints_];

    joint_damping_   = joint_damping_command_;
    joint_stiffness_ = joint_stiffness_command_;

    mFRI->GetMeasuredJointPositions(msrJntPos);
    mFRI->GetMeasuredJointTorques(msrJntTrq);

    for (int j = 0; j < n_joints_; j++)
    {

        joint_position_prev_[j] = joint_position_[j];
        joint_position_[j]      = (double)msrJntPos[j];
        joint_position_kdl_(j)  = joint_position_[j];
        joint_effort_[j]        = (double)msrJntTrq[j];


        joint_velocity_[j]      = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
        joint_stiffness_[j]     = joint_stiffness_command_[j];

        /*  joint_position_prev_[j]     = joint_position_[j];
        joint_position_kdl_(j)      = joint_position_[j];
        joint_velocity_[j]          = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
        joint_stiffness_[j]         = joint_stiffness_command_[j];*/
    }

    // ROS_INFO_STREAM_THROTTLE(0.1,"period.toSec(): " << period.toSec());
    // ROS_INFO_STREAM_THROTTLE(0.1,"LWR qdot: " << joint_velocity_[0] << " " << joint_velocity_[1] << " " << joint_velocity_[2] << " " << joint_velocity_[3] << " " << joint_velocity_[4] << " " << joint_velocity_[5] << " " << joint_velocity_[6]);


    /// FRI_STATE_MON || FRI_STATE_CMD
    mCurrentFRI_STATE = int2FRI_STATE(mFRI->GetFRIMode());

    /// FRI_CTRL_POSITION | FRI_CTRL_CART_IMP | FRI_CTRL_JNT_IMP | FRI_CTRL_OTHER
    mCurrentFRI_Control   = int2FRI_CTRL(mFRI->GetCurrentControlScheme());

}

void LWRRobot_FRI::write(ros::Time time, ros::Duration period)
{

    enforceLimits(period);

    if(mFRI->IsMachineOK())
    {

        if(mSwitched){
            ROS_INFO_STREAM_THROTTLE(1.0,"time: " << period.toSec());
            mFRI->WaitForKRCTick();
        }

        float newJntPosition[n_joints_];
        float newJntStiff[n_joints_];
        float newJntDamp[n_joints_];
        float newJntTorque[n_joints_];

        switch (getControlStrategy())
        {
        case NONE:
            break;
        case JOINT_POSITION:
            ROS_INFO_STREAM_THROTTLE(1.0,"case ==> JOINT_POSITION:");

            for (int j = 0; j < n_joints_; j++)
            {
                newJntPosition[j] = joint_position_command_[j];
                newJntStiff[j]    = joint_stiffness_command_[j];
                newJntDamp[j]     = joint_damping_command_[j];
            }
            mFRI->SetCommandedJointPositions(newJntPosition);
            mFRI->SetCommandedJointStiffness(newJntStiff);
            mFRI->SetCommandedJointDamping(newJntDamp);
            break;
        case CARTESIAN_IMPEDANCE:
            break;
        case JOINT_IMPEDANCE:
            ROS_INFO_STREAM_THROTTLE(1.0,"case ==> JOINT_IMPEDANCE:");

            for(int j=0; j < n_joints_; j++)
            {
                newJntPosition[j]     = joint_position_command_[j];
             //   newJntTorque[j]       = joint_effort_command_[j];
                newJntStiff[j]        = joint_stiffness_command_[j];
                newJntDamp[j]         = joint_damping_command_[j];
            }

            ROS_INFO_STREAM_THROTTLE(1.0,"D: " << newJntDamp[0]);
            ROS_INFO_STREAM_THROTTLE(1.0,"K: " << newJntStiff[0]);

            mFRI->SetCommandedJointPositions(newJntPosition);
            mFRI->SetCommandedJointStiffness(newJntStiff);
            mFRI->SetCommandedJointDamping(newJntDamp);
            //mFRI->SetCommandedJointTorques(newJntTorque);
            break;

        case JOINT_EFFORT:
            ROS_INFO_STREAM_THROTTLE(1.0,"case ==> JOINT_EFFORT:");
            for(int j=0; j < n_joints_; j++)
            {
                 newJntTorque[j] = joint_effort_command_[j];
                 newJntStiff[j]  = 0;
                 newJntDamp[j]   = 0;
            }
            ROS_INFO_STREAM_THROTTLE(1.0,"newJntTorque: " << newJntTorque[0] << " "
                                                                             << newJntTorque[1] << " "
                                                                             << newJntTorque[2] << " "
                                                                             << newJntTorque[3]
                                                                                );
            // mirror the position
            //mFRI->GetMeasuredJointPositions(joint_position_);
            //mFRI->SetCommandedJointPositions(joint_position_);
            mFRI->SetCommandedJointTorques(newJntTorque);
            mFRI->SetCommandedJointStiffness(newJntStiff);
            mFRI->SetCommandedJointDamping(newJntDamp);
            break;
        case GRAVITY_COMPENSATION:
            mFRI->GetMeasuredJointPositions(joint_position_);
            mFRI->SetCommandedJointPositions(joint_position_);
            break;
        }

    }
    return;
}

void LWRRobot_FRI::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    mSwitched=false;
    std::cout<< "DO_SWITCH" << std::endl;
    // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
        std::cout<< "hardware_interface: " << it->hardware_interface << std::endl;

        if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
        {
            std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_IMPEDANCE)" << std::endl;
            mDesired_control_strategy = JOINT_IMPEDANCE;
            break;
        }
        else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
        {
            std::cout << "Request to switch to hardware_interface::EffortJointInterface (JOINT_EFFORT)" << std::endl;
            mDesired_control_strategy = JOINT_EFFORT;
            break;
        }
    }
    for (int j = 0; j < n_joints_; ++j)
    {
        ///semantic Zero
        joint_position_command_[j] = joint_position_[j];
        joint_effort_command_[j] = 0.0;

        ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
        try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
        catch(const hardware_interface::HardwareInterfaceException&){}
        try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
        catch(const hardware_interface::HardwareInterfaceException&){}

        ///reset joint_limit_interfaces
        pj_sat_interface_.reset();
        pj_limits_interface_.reset();
    }

    std::cout<< "set control mode" << std::endl;

    if(mDesired_control_strategy == getControlStrategy())
    {
        std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
        SetControlMode(mDesired_control_strategy);
        setControlStrategy(mDesired_control_strategy);
        mSwitched=true;
        std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
}

bool LWRRobot_FRI::SetControlMode(ControlStrategy desiredMode){
    if(desiredMode ==  JOINT_POSITION){
        std::cout<< "Waiting for script..." << std::endl;
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return false;
        }
        std::cout<< "Robot set in position control mode" << std::endl;
    }
    else if(desiredMode == CARTESIAN_IMPEDANCE){
        std::cout<< "Waiting for script..." << std::endl;
        int result =  mFRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
    }
    else if(desiredMode == JOINT_IMPEDANCE){
        std::cout<< "desiredMode ==  JOINT_IMPEDANCE" << std::endl;
        mFRI->SetKRLBoolValue(0,true);
        std::cout<< "Waiting for script..." << std::endl;
        mFRI-> WaitForKRCTick();
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
        std::cout<< "Robot set in joint impedance mode." << std::endl;
    }
    else if(desiredMode == JOINT_EFFORT){
        mFRI->SetKRLBoolValue(0,true);
        std::cout<< "Waiting for script..." << std::endl;
        mFRI-> WaitForKRCTick();
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
        float newJntStiff[n_joints_];
        float newJntDamp[n_joints_];
        float newJntTorque[n_joints_];

        for (int j = 0; j < n_joints_; j++)
        {
            newJntDamp[j]     = 0;
            newJntStiff[j]    = 0;
            newJntTorque[j]   = 0;
        }
        mFRI->SetCommandedJointDamping(newJntDamp);
        mFRI->SetCommandedJointStiffness(newJntStiff);
        mFRI->SetCommandedJointTorques(newJntTorque);
        std::cout<< "Robot set in torque control mode." << std::endl;
    }
    else if(desiredMode == GRAVITY_COMPENSATION){
        std::cout<< "Waiting for script..." << std::endl;
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
        std::cout<< "Robot set in grav. comp. mode" << std::endl;
    }
    return true;
}

}
