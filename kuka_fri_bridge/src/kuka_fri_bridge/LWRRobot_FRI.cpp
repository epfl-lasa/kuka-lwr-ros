#include "kuka_fri_bridge/LWRRobot_FRI.h"
#include "LinuxAbstraction.h"
namespace kfb {

LWRRobot_FRI::LWRRobot_FRI(boost::shared_ptr<FastResearchInterface>& mFRI) :
    mFRI(mFRI),
    lwr_hw::LWRHW()
{

    mDesired_control_strategy = NONE;

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

   joint_damping_   = joint_damping_command_;
   joint_stiffness_ = joint_stiffness_command_;

    mFRI->GetMeasuredJointPositions(joint_position_);
    mFRI->GetMeasuredJointTorques(joint_effort_);

    for (int j = 0; j < n_joints_; j++)
    {
        joint_position_prev_[j]     = joint_position_[j];
        joint_position_kdl_(j)      = joint_position_[j];
        joint_velocity_[j]          = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
        joint_stiffness_[j]         = joint_stiffness_command_[j];
    }

    /// FRI_STATE_MON || FRI_STATE_CMD
    mCurrentFRI_STATE = int2FRI_STATE(mFRI->GetFRIMode());

    /// FRI_CTRL_POSITION | FRI_CTRL_CART_IMP | FRI_CTRL_JNT_IMP | FRI_CTRL_OTHER
    mCurrentFRI_Control   = int2FRI_CTRL(mFRI->GetCurrentControlScheme());

}

void LWRRobot_FRI::write(ros::Time time, ros::Duration period)
{

    std::string tmp =  fri_state2str(mCurrentFRI_STATE);
    ROS_INFO_DELAYED_THROTTLE(2.0,tmp.c_str());


    if (mCurrentFRI_STATE != FRI_STATE_CMD){
        return;
    }


    enforceLimits(period);

    float newJntPosition[n_joints_];
    float newJntStiff[n_joints_];
    float newJntDamp[n_joints_];
    float newJntAddTorque[n_joints_];



    switch (getControlStrategy())
    {
    case NONE:
       break;
    case JOINT_POSITION:
        for (int j = 0; j < n_joints_; j++)
        {
            newJntPosition[j] = joint_position_command_[j];
            newJntStiff[j]    = joint_stiffness_command_[j];
            newJntDamp[j]     = joint_damping_command_[j];
        }
        tmp = "JOINT_POSITION";


        ROS_INFO_STREAM_THROTTLE(1.0,tmp.c_str());
       // ROS_INFO_STREAM_THROTTLE(1.0,)
        mFRI->SetCommandedJointPositions(newJntPosition);
        mFRI->SetCommandedJointStiffness(newJntStiff);
        mFRI->SetCommandedJointDamping(newJntDamp);

        break;
    case CARTESIAN_IMPEDANCE:
        break;

    case JOINT_IMPEDANCE:
        tmp = "JOINT_IMPEDANCE (no torques)";

        for(int j=0; j < n_joints_; j++)
        {
            newJntPosition[j]     = joint_position_command_[j];
            newJntStiff[j]        = joint_stiffness_command_[j];
            newJntDamp[j]         = joint_damping_command_[j];

            /*if(std::fabs(newJntPosition[j] - joint_position_[j])/period.toSec() > 0.1  ){
                ROS_INFO("joint update too fast");
            }*/
        }

        //position_interface_.getHandle()

        ROS_INFO_STREAM_THROTTLE(1.0,tmp.c_str());

        mFRI->SetCommandedJointPositions(newJntPosition);
        mFRI->SetCommandedJointStiffness(newJntStiff);
        mFRI->SetCommandedJointDamping(newJntDamp);

          break;

    case JOINT_EFFORT:
        for(int j=0; j < n_joints_; j++)
        {
            newJntAddTorque[j] = joint_effort_command_[j];
        }
        tmp = "JOINT_EFFORT";
        ROS_INFO_DELAYED_THROTTLE(1.0,tmp.c_str());

        // mirror the position
        mFRI->GetMeasuredJointPositions(joint_position_);
        mFRI->SetCommandedJointPositions(joint_position_);
        mFRI->SetCommandedJointTorques(newJntAddTorque);
        // device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), NULL, NULL, newJntAddTorque, false);
        break;

    case JOINT_STIFFNESS:
        for(int j=0; j < n_joints_; j++)
        {
            newJntPosition[j] = joint_position_command_[j];
            newJntStiff[j] = joint_stiffness_command_[j];
        }
        mFRI->SetCommandedJointDamping(newJntPosition);
        mFRI->SetCommandedJointStiffness(newJntStiff);
        //        device_->doJntImpedanceControl(newJntPosition, newJntStiff, NULL, NULL, false);
        break;

    case GRAVITY_COMPENSATION:
        mFRI->GetMeasuredJointPositions(joint_position_);
        mFRI->SetCommandedJointPositions(joint_position_);
        // device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), NULL, NULL, NULL, false);
        break;
    }
    return;
}

void LWRRobot_FRI::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{

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


  //  if(mCurrentFRI_STATE != FRI_STATE_CMD){
        mFRI->WaitForKRCTick();
   // }

    /// NONE| JOINT_POSITION | CARTESIAN_IMPEDANCE | JOINT_IMPEDANCE | JOINT_EFFORT | JOINT_STIFFNESS | GRAVITY_COMPENSATION

    //if(mCurrentFRI_Control != mDesired_control_strategy){
       std::cout<< "set control mode" << std::endl;
       SetControlMode(mDesired_control_strategy);
       setControlStrategy(mDesired_control_strategy);
  //  }


    // if(desired_strategy == getControlStrategy())
    // {
    //     std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
    // }
    //else
    // {
    //mFRI->WaitForKRCTick();

    //stopFRI();

    // send to KRL the new strategy
    //if( desired_strategy == JOINT_POSITION )
    //device_->setToKRLInt(0, JOINT_POSITION);
    // else if( desired_strategy >= JOINT_IMPEDANCE)
    //device_->setToKRLInt(0, JOINT_IMPEDANCE);

    // startFRI();


    // std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    // }
}

void LWRRobot_FRI::actual_switch(){

    mCurrentFRI_STATE = int2FRI_STATE(mFRI->GetFRIMode());
    mCurrentFRI_Control   = int2FRI_CTRL(mFRI->GetCurrentControlScheme());

    if(mCurrentFRI_STATE != FRI_STATE_CMD){
        mFRI->WaitForKRCTick();
    }

    if(mCurrentFRI_Control != mDesired_control_strategy){

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

        SetControlMode(mDesired_control_strategy);
        setControlStrategy(mDesired_control_strategy);
    }

}


bool LWRRobot_FRI::SetControlMode(ControlStrategy desiredMode){

    if(desiredMode ==  JOINT_POSITION){

        //  mNCConsole.Print("Waiting for script...");
        //  mNCConsole.Render();
        //  std::cout<< "Waiting for script..." << std::endl;
        // mFRI->GetMeasuredJointPositions(jnt2);
        // SetCommandedJPos(jnt2);

        std::cout<< "desiredMode ==  JOINT_POSITION" << std::endl;

        mFRI-> WaitForKRCTick();
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return false;
        }
        //mNCConsole.Print("Robot set in position control mode");
        std::cout<< "Robot set in position control mode" << std::endl;

        //  nControl = Robot::CTRLMODE_POSITION;
    }
    else if(desiredMode == CARTESIAN_IMPEDANCE){
        //mNCConsole.Print("Waiting for script...");
        //mNCConsole.Render();
        std::cout<< "Waiting for script..." << std::endl;
        mFRI->StopRobot();
        //mFRI->GetMeasuredCartPose(cart2);
        // mFRI->SetCommandedCartPose(cart2);
        mFRI-> WaitForKRCTick();
        int result =  mFRI->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
        /*  for(int i=0;i<3; i++)
      cart_imp_params[i] =  mCurrCStiff_pos;
    for(int i=3;i<6; i++)
      cart_imp_params[i] =  mCurrCStiff_or;

    mFRI->SetCommandedCartStiffness(cart_imp_params);

    for(int i=0;i<3; i++)
      cart_imp_params[i] =  mCurrCDamp_pos;
    for(int i=3;i<6; i++)
      cart_imp_params[i] =  mCurrCDamp_or;

    mFRI->SetCommandedCartDamping(cart_imp_params);


    mNCConsole.Print("Robot set in cartesian impedance control mode");


    nControl = Robot::CTRLMODE_CARTIMPEDANCE;*/

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
        /*
    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i] =  mCurrJDamp;
    mFRI->SetCommandedJointDamping(jnt2);

    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i] =  mCurrJStiff;
    mFRI->SetCommandedJointStiffness(jnt2);

    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i]=0.;
    mFRI->SetCommandedJointTorques(jnt2);

    nControl = Robot::CTRLMODE_JOINTIMPEDANCE;
    mNCConsole.Print("Robot set in joint impedance mode.");*/
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
        /*
    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i] =  0.0;
    mFRI->SetCommandedJointDamping(jnt2);

    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i] =  0.0;
    mFRI->SetCommandedJointStiffness(jnt2);

    for(int i=0;i<LBR_MNJ; i++)
      jnt2[i]=0.0;
    mFRI->SetCommandedJointTorques(jnt2);

    nControl = Robot::CTRLMODE_TORQUE;
    mNCConsole.Print("Robot set in torque control mode.");*/
        std::cout<< "Robot set in torque control mode." << std::endl;

    }
    else if(desiredMode == GRAVITY_COMPENSATION){
        std::cout<< "Waiting for script..." << std::endl;
        mFRI->SetKRLBoolValue(0,true);
        mFRI-> WaitForKRCTick();
        int result =  mFRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL, FRI_CONN_TIMEOUT_SEC);
        if(result != EOK && result != EALREADY)
        {
            std::cout<<"Error: "<<result<<std::endl;
            return 1;
        }
        /*
    Vector v = mLWRRobot-> GetJointStiffness();
    for(int i=0;i<LBR_MNJ; i++)
      if(mLWRRobot->GetGravComp(i))
        jnt2[i] = 0;
      else
        jnt2[i] = v(i);

    v = mLWRRobot-> GetJointDamping();
    float jdamp[LBR_MNJ];
    for(int i=0;i<LBR_MNJ; i++)
      if(mLWRRobot->GetGravComp(i))
        jdamp[i] = 0;
      else
        jdamp[i] = v(i);


    mFRI->SetCommandedJointStiffness(jnt2);
    mFRI->SetCommandedJointDamping(jdamp);

    nControl = Robot::CTRLMODE_GRAVITYCOMPENSATION;
    mNCConsole.Print("Robot set in grav. comp. mode");*/

        std::cout<< "Robot set in grav. comp. mode" << std::endl;
    }
    // nMode = MODE_NONE;
    return true;
}


void LWRRobot_FRI::startFRI()
{
    // wait until FRI enters in command mode
    std::cout << "Waiting for good communication quality..." << std::endl;
    // while( device_->getQuality() != FRI_QUALITY_OK ){};
    // device_->setToKRLInt(1, 1);

    // std::cout << "Waiting for command mode..." << std::endl;
    // while ( device_->getFrmKRLInt(1) != 1 )
    // {
    // std::cout << "device_->getState(): " << device_->getState() << std::endl;
    // device_->setToKRLInt(1, 1);
    // usleep(1000000);
    // }
    return;
}

void LWRRobot_FRI::stopFRI()
{
    // wait until FRI enters in command mode
    mFRI->SetKRLIntValue(1,0);

    // device_->setToKRLInt(1, 0);
    std::cout << "Waiting for monitor mode..." << std::endl;
    mFRI->WaitForKRCTick();
    // while ( device_->getFrmKRLInt(1) != 0 ){}
    // {
    // std::cout << "device_->getState(): " << device_->getState() << std::endl;
    // std::cout << "Waiting for monitor mode..." << std::endl;
    // device_->setToKRLInt(1, 0);
    // usleep(1000000);
    // }
    //return;
    //}

}

}
