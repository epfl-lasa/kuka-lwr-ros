#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_kinematics_torq.h>

namespace lwr_controllers {

JointKinematicsTorq::JointKinematicsTorq() {}

JointKinematicsTorq::~JointKinematicsTorq() {}

bool JointKinematicsTorq::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{

    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);


    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());
    K_tmp.resize(kdl_chain_.getNrOfJoints());
    D_tmp.resize(kdl_chain_.getNrOfJoints());
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        D_(i) = 0;
        K_(i) = 0;
        D_tmp(i) = 0;
        K_tmp(i) = 0;
    }

    ROS_INFO("Loading resource for stiffness values");
    // Get joint handles for all of the joints in the chain
  /*  for(std::size_t i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_stiffness"));
    }
    */


    qdot_msg.data.resize(kdl_chain_.getNrOfJoints());

    joint_filt_.resize(kdl_chain_.getNrOfJoints());

    tau_cmd_.resize(kdl_chain_.getNrOfJoints());

    x_linear_des_.resize(3);
    x_linear_msr_.resize(3);
    x_msr_.resize(6);
    F_ee_des_.resize(6);

    /// one_task_inverse_kinematics stuff
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
    id_solver_gravity_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

    /// Passive dynamical system

    passive_ds_controller.reset(new DSController(3,50.0,50.0));

    J_.resize(kdl_chain_.getNrOfJoints());

    // get joint positions
    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_.q(i)    = joint_msr_.q(i);
        joint_des_.qdot(i) = 0;
    }

    sub_command_grav_      = nh_.subscribe("command_grav",     1, &JointKinematicsTorq::command_grav,         this);
    sub_command_joint_pos_ = nh_.subscribe("command_joint_pos",1, &JointKinematicsTorq::command_joint_pos,    this);
    sub_command_pose_      = nh_.subscribe("command_pos",      1, &JointKinematicsTorq::command_cart_pos,     this);
    sub_command_vel_       = nh_.subscribe("command_vel",      1, &JointKinematicsTorq::command_cart_vel,     this);
    sub_command_orient_    = nh_.subscribe("command_orient",   1 ,&JointKinematicsTorq::command_orient,       this);
    sub_stiff_             = nh_.subscribe("stiffness",        1, &JointKinematicsTorq::setStiffness,         this);
    sub_damp_              = nh_.subscribe("damping",          1, &JointKinematicsTorq::setDamping,           this);
    pub_qdot_              = n.advertise<std_msgs::Float64MultiArray>("qdot",10);

    nd1 = ros::NodeHandle("D_param");
    nd2 = ros::NodeHandle("K_param");
    nd3 = ros::NodeHandle("D_all_param");
    nd4 = ros::NodeHandle("K_all_param");
    nd5 = ros::NodeHandle("ds_param");

    dynamic_server_D_param.reset(new        dynamic_reconfigure::Server< lwr_controllers::damping_paramConfig   >(nd1));
    dynamic_server_K_param.reset(new        dynamic_reconfigure::Server< lwr_controllers::stiffness_paramConfig >(nd2));

    dynamic_server_D_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig   >(nd3));
    dynamic_server_K_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig >(nd4));

    dynamic_server_ds_param.reset(new   dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));


    dynamic_server_D_param->setCallback(    boost::bind(&JointKinematicsTorq::damping_callback,     this, _1, _2));
    dynamic_server_K_param->setCallback(    boost::bind(&JointKinematicsTorq::stiffness_callback,   this, _1, _2));
    dynamic_server_ds_param->setCallback(   boost::bind(&JointKinematicsTorq::ds_param_callback,    this, _1, _2));

    dynamic_server_D_all_param->setCallback(  boost::bind(&JointKinematicsTorq::damping_all_callback,     this, _1, _2));
    dynamic_server_K_all_param->setCallback(  boost::bind(&JointKinematicsTorq::stiffness_all_callback,     this, _1, _2));

    publish_rate_          = 100;

    /// Filter

    joint_cddynamics.reset(new motion::CDDynamics(7,1e-6,1));
    motion::Vector velLimits(7);
    for(std::size_t i = 0; i < 7; i++){
        velLimits(i)  = 0.5; // x ms^-1
    }
    joint_cddynamics->SetVelocityLimits(velLimits);

    std::cout<< "finished init" << std::endl;
    return true;
}

void JointKinematicsTorq::starting(const ros::Time& time)
{
    ROS_INFO("JointKinematicsTorqu::starting");
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
        tau_cmd_(i)       = 0.0;
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }

    {
        lwr_controllers::damping_paramConfig config;
        dynamic_server_D_param->getConfigDefault(config);
        damping_callback(config,0);
    }
    {
        lwr_controllers::stiffness_paramConfig config;
        dynamic_server_K_param->getConfigDefault(config);
        ROS_INFO_STREAM("config K: " << config.K_0_joint);
        stiffness_callback(config,0);
    }

    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();

    ctrl_mode = GRAV_COMP;
    cart_type = VELOCITY_PASSIVE_DS;
    std::cout<< "finished starting" << std::endl;
}

void JointKinematicsTorq::update(const ros::Time& time, const ros::Duration& period)
{

    // get measured joint positions and velocity
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        qdot_msg.data[i]          = joint_msr_.qdot(i);
    }
    pub_qdot_.publish(qdot_msg);

    jnt_to_jac_solver_->JntToJac(joint_msr_.q,J_);
    fk_pos_solver_->JntToCart(joint_msr_.q, x_);

    x_orient_[0][0] = x_.M(0,0);
    x_orient_[0][1] = x_.M(0,1);
    x_orient_[0][2] = x_.M(0,2);

    x_orient_[1][0] = x_.M(1,0);
    x_orient_[1][1] = x_.M(1,1);
    x_orient_[1][2] = x_.M(1,2);

    x_orient_[2][0] = x_.M(2,0);
    x_orient_[2][1] = x_.M(2,1);
    x_orient_[2][2] = x_.M(2,2);

    switch(ctrl_mode)
    {
    case CART_VELOCITIY:
    {
        ROS_INFO_THROTTLE(1.0,"CART_VELOCITY");
        if       (cart_type == VELOCITY_OPEN_LOOP)
        {
            velocity_open_loop_update(period);
        }else if (cart_type == VELOCITY_PASSIVE_DS){
            passive_ds_update();
        }
        break;
    }
    case JOINT_POSITION:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"JOINT_POSITION");
        joint_cddynamics->SetDt(period.toSec());
        joint_cddynamics->SetTarget(joint_des_.q.data);
        joint_cddynamics->Update();
        joint_cddynamics->GetState(joint_filt_.q.data);

        for(std::size_t i = 0; i < joint_handles_.size();i++){
            tau_cmd_(i)  =   -D_(i) * (joint_msr_.qdot(i)) - K_(i) * (joint_msr_.q(i)  - joint_filt_.q(i));
        }
        break;
    }
    case GRAV_COMP:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"GRAV_COMP");
        for(std::size_t i = 0; i < joint_handles_.size();i++){
            tau_cmd_(i)         = 0;
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
        }
        break;
    }
    default:    // same as grav-comp
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"NONE");
        for(std::size_t i = 0; i < joint_handles_.size();i++){
            tau_cmd_(i)         = 0;
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
        }
        break;
    }
    }

    //std::cout << J_.data << std::endl;

    //Compute control law
    //id_solver_gravity_->JntToGravity( q_msr_ , tau_gravity_ );
    //  for(size_t i=0; i<joint_handles_.size(); i++) {
    //  tau_cmd_(i) = K_(i) * (q_des_(i) - q_msr_(i)) + D_(i)*dotq_msr_.qdot(i) + tau_des_(i) + tau_gravity_(i);
    //  tau_gravity_(i);
    //  tau_cmd_(i) = - D_(i) * (dotq_msr_.qdot(i) - joint_des_states_.qdot(i));
    // }

    ROS_INFO_STREAM_THROTTLE(1.0,"K: " << K_(0) << " " << K_(1) << " " << K_(2) << " " << K_(3) << " " << K_(4) << " " << K_(5) << " " << K_(6));
    ROS_INFO_STREAM_THROTTLE(1.0,"D: " << D_(0) << " " << D_(1) << " " << D_(2) << " " << D_(3) << " " << D_(4) << " " << D_(5) << " " << D_(6));
    ROS_INFO_STREAM_THROTTLE(1.0,"tau_cmd_: " << tau_cmd_(0) << " " <<  tau_cmd_(1) << " " <<  tau_cmd_(2) << " " << tau_cmd_(3) << " " <<  tau_cmd_(4) << " " <<  tau_cmd_(5) << " " << tau_cmd_(6));

    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(tau_cmd_(i));
    }
    //std::cout<< "update #4 "<< std::endl;


}


void JointKinematicsTorq::velocity_open_loop_update(const ros::Duration& period){
    ROS_INFO_STREAM_THROTTLE(1.0,"CART_VELOCITIY");
    ROS_INFO_STREAM_THROTTLE(1.0,"x_des_vel_: " << x_des_vel_.vel(0) << " "  << x_des_vel_.vel(1) << " "  << x_des_vel_.vel(2) << " " << x_des_vel_.rot(0) <<  " " << x_des_vel_.rot(1) << " "  << x_des_vel_.rot(2));
    ik_vel_solver_->CartToJnt(joint_msr_.q,x_des_vel_,joint_des_.qdot);
    for (std::size_t i = 0; i < joint_handles_.size(); i++){
        // integrating q_dot -> getting q (Euler method)
        joint_des_.q(i) = joint_des_.q(i) + period.toSec()*joint_des_.qdot(i);
        // joint position and velocity error to torque
        tau_cmd_(i)     =   -D_(i) * (joint_msr_.qdot(i) - joint_des_.qdot(i)) - K_(i) * (joint_msr_.q(i)  - joint_des_.q(i));

    }
}

void JointKinematicsTorq::passive_ds_update(){
    ROS_INFO_STREAM_THROTTLE(1.0,"CART_VELOCITY_DS");
    /// set desired linear velocity
    x_linear_des_(0) = x_des_vel_(0);
    x_linear_des_(1) = x_des_vel_(1);
    x_linear_des_(2) = x_des_vel_(2);

    /// 3 linear & 3 angular velocities
    x_msr_ = J_.data * joint_msr_.qdot.data;

    /// set measured linear velocity
    x_linear_msr_(0) = x_msr_(0);
    x_linear_msr_(1) = x_msr_(1);
    x_linear_msr_(2) = x_msr_(2);

    // DSController.Update();
    passive_ds_controller->Update(x_linear_msr_,x_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)

    F_ee_des_(0) = F_linear_des_(0);
    F_ee_des_(1) = F_linear_des_(1);
    F_ee_des_(2) = F_linear_des_(2);

    F_ee_des_(3) = -0.1 * x_msr_(3);
    F_ee_des_(4) = -0.1 * x_msr_(4);
    F_ee_des_(5) = -0.1 * x_msr_(5);

    // damp any rotational motion
//	Vector control_torque = cart_vel_r*(-rot_damping_);
// 	Matrix3 err_orient;
 //	err_orient = cart_orient_*cart_ref_orient_.Transpose();
    tf::Matrix3x3 err_orient = x_orient_ * x_des_orient_.transpose();

    //	Vector3 err_orient_axis;
    //	realtype err_orient_angle;
    double err_orient_angle;
    double rot_stiffness = 10;
    tf::Quaternion q;
    err_orient.getRotation(q);
    //	err_orient_axis = err_orient.GetNormRotationAxis();
    tf::Vector3 err_orient_axis = q.getAxis();
    //	err_orient_angle = err_orient.GetRotationAngle();
    err_orient_angle            = q.getAngle();

//	// rotational stiffness. This is correct sign and everything!!!! do not mess with this!
//	control_torque += err_orient_axis*err_orient_angle*(-rot_stiffness_);
    tf::Vector3 torque_orient = err_orient_axis * err_orient_angle * (-rot_stiffness);

    F_ee_des_(3) =  F_ee_des_(3) + torque_orient.getX();
    F_ee_des_(4) =  F_ee_des_(4) + torque_orient.getY();
    F_ee_des_(5) =  F_ee_des_(5) + torque_orient.getZ();


//	// combine in one wrench vector
//	Vector cartesian_control(6);
//	cartesian_control.SetSubVector(0,control_force);
//	cartesian_control.SetSubVector(3,control_torque);

    tau_cmd_.data = J_.data.transpose() * F_ee_des_;

}


void JointKinematicsTorq::command(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if (msg->data.size() != joint_handles_.size()) {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        //  for (unsigned int j = 0; j < joint_handles_.size(); ++j)
        //       q_des_(j) = msg->data[j];
    }

}

void JointKinematicsTorq::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO_THROTTLE(1.0,"command cart pos");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));
    //ctrl_type = CART_POSITION;
    //x_des_    = frame_des_;
    cmd_flag_ = true;
}

void JointKinematicsTorq::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    ROS_INFO_THROTTLE(1.0,"command_cart_vel");
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    ctrl_mode = CART_VELOCITIY;
    cmd_flag_ = true;

}

void JointKinematicsTorq::command_grav(const std_msgs::Bool& msg){
    ROS_INFO_THROTTLE(1.0,"command_grav");
    if(msg.data == true)
    {
        ROS_INFO("Starting Gravity compensation");
        ctrl_mode = GRAV_COMP;
    }else{
        ROS_INFO("Stopping Gravity compensation");
        for(std::size_t i = 0; i < 7; i++){
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
        }
        joint_cddynamics->SetState(joint_msr_.q.data);
        ctrl_mode = JOINT_POSITION;
    }
}

void JointKinematicsTorq::command_set_cart_type(const std_msgs::Int32& msg){

    switch(msg.data)
    {
    case 0:
    {
        cart_type = VELOCITY_OPEN_LOOP;
        break;
    }
    case 1:
    {
        cart_type = VELOCITY_PASSIVE_DS;
        break;
    }
    default:
    {

    }
    }

}

void JointKinematicsTorq::command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if (msg->data.size() != joint_handles_.size()) {
        ROS_INFO("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        ROS_INFO("Joint impedance setting desired position");
        for (std::size_t j = 0; j < joint_handles_.size(); ++j){
            joint_des_.q(j)     = msg->data[j];
            joint_des_.qdot(j)  = 0.0;
        }

        joint_cddynamics->SetState(joint_msr_.q.data);
        ctrl_mode = JOINT_POSITION;
    }

}

void JointKinematicsTorq::command_orient(const std_msgs::Float64MultiArray::ConstPtr &msg){
    x_des_orient_.setRPY(msg->data[0],msg->data[1],msg->data[2]);
}

void JointKinematicsTorq::setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            K_(i)       = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Stiffness Num of Joint handles = %lu", joint_handles_.size());
    }
}

void JointKinematicsTorq::setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            D_(i)    = msg->data[i];
        }
    }else
    {
        ROS_INFO("Damping Num of Joint handles = %lu", joint_handles_.size());
    }
}

void JointKinematicsTorq::damping_callback(damping_paramConfig &config, uint32_t level){
    D_(0) = config.damp_0_joint;
    D_(1) = config.damp_1_joint;
    D_(2) = config.damp_2_joint;
    D_(3) = config.damp_3_joint;
    D_(4) = config.damp_4_joint;
    D_(5) = config.damp_5_joint;
    D_(6) = config.damp_6_joint;
}

void JointKinematicsTorq::stiffness_callback(lwr_controllers::stiffness_paramConfig& config, uint32_t level){
    K_(0) = config.K_0_joint;
    K_(1) = config.K_1_joint;
    K_(2) = config.K_2_joint;
    K_(3) = config.K_3_joint;
    K_(4) = config.K_4_joint;
    K_(5) = config.K_5_joint;
    K_(6) = config.K_6_joint;
}


void JointKinematicsTorq::damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level){
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        D_(i)   = config.D;
    }

}

void JointKinematicsTorq::stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level){
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        K_(i)   = config.K;
    }
}

void JointKinematicsTorq::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
}


}                                                           // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointKinematicsTorq, controller_interface::ControllerBase)
