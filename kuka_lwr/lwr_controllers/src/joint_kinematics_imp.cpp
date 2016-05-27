#include <lwr_controllers/joint_kinematics_imp.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <cmath>

namespace lwr_controllers
{

JointKinematiscImp::JointKinematiscImp() {}

JointKinematiscImp::~JointKinematiscImp() {}

bool JointKinematiscImp::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{

    std::cout<<"   "<<std::endl;
    if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
    {
        ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
        return false;
    }

    K_.resize(7);
    D_.resize(7);
    K_tmp.resize(7);

    for (unsigned int i = 0; i < joint_handles_.size(); ++i){
        if ( !n.getParam("stiffness_gains", K_(i) ) ){
            ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (unsigned int i = 0; i < joint_handles_.size(); ++i){
        if ( !n.getParam("damping_gains", D_(i)) ){
            ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    // Get joint handles for all of the joints in the chain
    for(std::size_t i = 0; i < 7; i++)
    {
        joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_stiffness"));
        joint_handles_damping.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_damping"));
      //  joint_handles_stiff_two.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_stiffness"));
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

    q_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    // get joint positions
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i) = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_.q(i) = joint_msr_.q(i);
    }

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_.q, x_);

    //Desired posture is the current one
    x_des_ = x_;

    cmd_flag_ = false;

    sub_command_grav_      = nh_.subscribe("command_grav",     1, &JointKinematiscImp::command_grav,         this);
    sub_command_joint_pos_ = nh_.subscribe("command_joint_pos",1, &JointKinematiscImp::command_joint_pos,    this);
    sub_command_pose_      = nh_.subscribe("command_pos",      1, &JointKinematiscImp::command_cart_pos,     this);
    sub_command_vel_       = nh_.subscribe("command_vel",      1, &JointKinematiscImp::command_cart_vel,     this);
    sub_stiff_             = nh_.subscribe("stiffness",        1, &JointKinematiscImp::setStiffness,         this);
    sub_damp_              = nh_.subscribe("damping",          1, &JointKinematiscImp::setDamping,           this);
    publish_rate_          = 50;


    joint_cddynamics = std::unique_ptr<motion::CDDynamics>(new motion::CDDynamics(7,1e-6,1));

    motion::Vector velLimits(7);
    for(std::size_t i = 0; i < 7; i++){
        velLimits(i)  = 0.2; // x ms^-1
    }
    joint_cddynamics->SetVelocityLimits(velLimits);
    q_des_.data.resize(7);
    q_target_.resize(7);

    return true;
}

void JointKinematiscImp::starting(const ros::Time& time)
{
    ROS_INFO("starting on one task inverse kinematics");
    cmd_flag_ = false;
    // get joint positions
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)             = joint_handles_[i].getPosition();
        joint_des_.q(i)             = joint_msr_.q(i);
        joint_des_.qdot(i)          = 0.0;
        q_target_(i)                = joint_msr_.q(i);
        D_(i)                       = 2;
        K_tmp(i)                    = K_(i);
        joint_handles_[i].setCommand(joint_des_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
        joint_handles_damping[i].setCommand(D_(i));
    }

    joint_cddynamics->SetState(joint_msr_.q.data);


    x_des_vel_  = KDL::Twist::Zero();
    ctrl_type   = JOINT_POSITION;
    // initialize time
    last_publish_time_ = time;

}

void JointKinematiscImp::update(const ros::Time& time, const ros::Duration& period)
{

    // get joint positions and setting default values
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
    }

    ROS_INFO_THROTTLE(1.0,"JointKinematiscImp");

    if(ctrl_type == JOINT_POSITION)
    {
        ROS_INFO_THROTTLE(3.0,"JOINT_POSITION");

        for(std::size_t i = 0; i < 7;i ++){
            joint_des_.qdot(i) = 0;
            joint_des_.q(i)    = q_target_(i);
        }
/*
        ROS_INFO_STREAM_THROTTLE(1.0,"msr: " << joint_msr_states_.q(0) << " "
                                 << joint_msr_states_.q(1) << " "
                                 << joint_msr_states_.q(2) << " "
                                 << joint_msr_states_.q(3) << " "
                                 << joint_msr_states_.q(4) << " "
                                 << joint_msr_states_.q(5) << " "
                                 << joint_msr_states_.q(6) << " "
                                 );
        ROS_INFO_STREAM_THROTTLE(1.0,"tar: " << joint_msr_states_.q(0) << " "
                                 << joint_des_states_.q(1) << " "
                                 << joint_des_states_.q(2) << " "
                                 << joint_des_states_.q(3) << " "
                                 << joint_des_states_.q(4) << " "
                                 << joint_des_states_.q(5) << " "
                                 << joint_des_states_.q(6) << " "
                                 );*/
        joint_cddynamics->SetDt(period.toSec());
        joint_cddynamics->SetTarget(joint_des_.q.data);
        joint_cddynamics->Update();
        joint_cddynamics->GetState(joint_des_.q.data);

    }else if(ctrl_type == GRAV_COMP){
        ROS_INFO_THROTTLE(1.0,"GRAV_COMP");
         for(std::size_t i = 0; i < 7;i ++){
            joint_des_.q(i)    = joint_msr_.q(i);
            joint_des_.qdot(i) = 0;
            K_(i)              = 0;
         }
    }else if(ctrl_type == CART_VELOCITIY){
        ROS_INFO_THROTTLE(1.0,"CART_VELOCITY");
        ik_vel_solver_->CartToJnt(joint_msr_.q,x_des_vel_,joint_des_.qdot);
    }else{
        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_.q, J_);

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_.q, x_);

        // end-effector position error
        x_err_.vel = x_des_.p - x_.p;

        // getting quaternion from rotation matrix
        x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
        x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

        skew_symmetric(quat_des_.v, skew_);

        for (int i = 0; i < skew_.rows(); i++)
        {
            v_temp_(i) = 0.0;
            for (int k = 0; k < skew_.cols(); k++)
                v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
        }

        // end-effector orientation error
        x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

        // computing q_dot
        for (int i = 0; i < J_pinv_.rows(); i++)
        {
            joint_des_.qdot(i) = 0.0;
            for (int k = 0; k < J_pinv_.cols(); k++)
                joint_des_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7

        }

        if (Equal(x_, x_des_, 0.005) && (ctrl_type == CART_POSITION))
        {
            ROS_INFO("On target");
            cmd_flag_ = false;
        }

    }



    // integrating q_dot -> getting q (Euler method)
    for (unsigned int i = 0; i < joint_handles_.size(); i++){
        joint_des_.q(i) = joint_des_.q(i) + period.toSec()*joint_des_.qdot(i);
    }


    // joint limits saturation
    /* for (int i =0;  i < joint_handles_.size(); i++)
    {
        if (joint_des_states_.q(i) < joint_limits_.min(i))
            joint_des_states_.q(i) = joint_limits_.min(i);
        if (joint_des_states_.q(i) > joint_limits_.max(i))
            joint_des_states_.q(i) = joint_limits_.max(i);
    }
*/


    // set controls for joints
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        D_(i) = 2;
        joint_handles_[i].setCommand(joint_des_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
        joint_handles_damping[i].setCommand(D_(i));
    }

}

void JointKinematiscImp::stopping(const ros::Time& /*time*/){
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)      = joint_handles_[i].getPosition();
        joint_des_.q(i)      =  joint_msr_.q(i);
        joint_des_.qdot(i)   = 0.0;
        joint_handles_[i].setCommand(joint_des_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
    }

    cmd_flag_ = true;
    ctrl_type = JOINT_POSITION;
}

void JointKinematiscImp::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO_THROTTLE(1.0,"command cart pos");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));
    ctrl_type = CART_POSITION;
    x_des_    = frame_des_;
    cmd_flag_ = true;
}

void JointKinematiscImp::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    ROS_INFO_THROTTLE(1.0,"command_cart_vel");
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    ctrl_type = CART_VELOCITIY;
    cmd_flag_ = true;

}

void JointKinematiscImp::command_grav(const std_msgs::Bool& msg){
    ROS_INFO_THROTTLE(1.0,"command_grav");
    if(msg.data == true)
    {
        ROS_INFO("Starting Gravity compensation");
        for(std::size_t i = 0; i < 7; i++){
            K_tmp(i) = K_(i);
            K_(i)    = 0;
        }
        ctrl_type = GRAV_COMP;
    }else{
        ROS_INFO("Stopping Gravity compensation");
        for(std::size_t i = 0; i < 7; i++){
            K_(i)        = K_tmp(i);
            q_target_(i) = joint_msr_.q(i);
        }
        joint_cddynamics->SetState(joint_msr_.q.data);
        ctrl_type = JOINT_POSITION;
    }

}

void JointKinematiscImp::command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        ROS_INFO("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        ROS_INFO("Joint impedance setting desired position");
        for (unsigned int j = 0; j < joint_handles_.size(); ++j){
            q_target_(j) = msg->data[j];
            joint_des_.qdot(j) = 0.0;
        }

        joint_cddynamics->SetState(joint_msr_.q.data);

        ctrl_type = JOINT_POSITION;
        cmd_flag_ = true;
    }

}

void JointKinematiscImp::setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            K_(i)       = msg->data[i];
            K_tmp(i)    = K_(i);
        }
    }
    else
    {
        ROS_INFO("Stiffness Num of Joint handles = %lu", joint_handles_.size());
    }
}

void JointKinematiscImp::setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            D_(i) = msg->data[i];
        }
    }else
    {
        ROS_INFO("Damping Num of Joint handles = %lu", joint_handles_.size());
    }
}

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::JointKinematiscImp, controller_interface::ControllerBase)
