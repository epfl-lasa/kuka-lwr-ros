#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/transform_broadcaster.h>
#include "lwr_controllers/joint_controllers.h"

namespace lwr_controllers {

JointControllers::JointControllers() {}

JointControllers::~JointControllers() {}

bool JointControllers::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{

    KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n);


    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());
    K_tmp.resize(kdl_chain_.getNrOfJoints());
    D_tmp.resize(kdl_chain_.getNrOfJoints());
    K_cmd.resize(kdl_chain_.getNrOfJoints());
    D_cmd.resize(kdl_chain_.getNrOfJoints());
    q_target_.resize(kdl_chain_.getNrOfJoints());

    pos_cmd_.resize(kdl_chain_.getNrOfJoints());

    for(unsigned int i=0; i < joint_handles_.size(); i++)
    {
        D_(i) = 0;
        K_(i) = 0;
        D_tmp(i) = 0;
        K_tmp(i) = 0;
    }

    ROS_INFO("Loading resource for stiffness values");
    // Get joint handles for all of the joints in the chain
    for(std::size_t i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()  + "_stiffness"));
        joint_handles_damping.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()    + "_damping"));
        joint_handles_torque.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()     + "_torque"));
    }



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
    for(std::size_t i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_.q(i)    = joint_msr_.q(i);
        joint_des_.qdot(i) = 0;
        pos_cmd_(i)      = joint_des_.q(i);

    }

    sub_command_grav_      = nh_.subscribe("command_grav",     1, &JointControllers::command_grav,         this);
    sub_command_joint_pos_ = nh_.subscribe("command_joint_pos",1, &JointControllers::command_joint_pos,    this);
    sub_command_pose_      = nh_.subscribe("command_pos",      1, &JointControllers::command_cart_pos,     this);
    sub_command_vel_       = nh_.subscribe("command_vel",      1, &JointControllers::command_cart_vel,     this);
    sub_command_orient_    = nh_.subscribe("command_orient",   1 ,&JointControllers::command_orient,       this);
    sub_stiff_             = nh_.subscribe("stiffness",        1, &JointControllers::setStiffness,         this);
    sub_damp_              = nh_.subscribe("damping",          1, &JointControllers::setDamping,           this);
    pub_qdot_              = n.advertise<std_msgs::Float64MultiArray>("qdot",10);

    nd1 = ros::NodeHandle("D_param");
    nd2 = ros::NodeHandle("K_param");
    nd3 = ros::NodeHandle("D_all_param");
    nd4 = ros::NodeHandle("K_all_param");
    nd5 = ros::NodeHandle("ds_param");
    nd6 = ros::NodeHandle("rot_stiffness");


    dynamic_server_D_param.reset(new        dynamic_reconfigure::Server< lwr_controllers::damping_paramConfig   >(nd1));
    dynamic_server_K_param.reset(new        dynamic_reconfigure::Server< lwr_controllers::stiffness_paramConfig >(nd2));

    dynamic_server_D_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig   >(nd3));
    dynamic_server_K_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig >(nd4));

    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));

    dynamic_server_rot_stiffness_param.reset(new   dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig>(nd6));



    dynamic_server_D_param->setCallback(     boost::bind(&JointControllers::damping_callback,      this, _1, _2));
    dynamic_server_K_param->setCallback(     boost::bind(&JointControllers::stiffness_callback,    this, _1, _2));
    dynamic_server_ds_param->setCallback(    boost::bind(&JointControllers::ds_param_callback,     this, _1, _2));

    dynamic_server_D_all_param->setCallback( boost::bind(&JointControllers::damping_all_callback,  this, _1, _2));
    dynamic_server_K_all_param->setCallback( boost::bind(&JointControllers::stiffness_all_callback,this, _1, _2));

    dynamic_server_rot_stiffness_param->setCallback( boost::bind(&JointControllers::rot_stiffness_callback,this, _1, _2));


    publish_rate_          = 100;

    /// Filter

    joint_cddynamics.reset(new motion::CDDynamics(7,1e-6,1));
    motion::Vector velLimits(7);
    for(std::size_t i = 0; i < 7; i++){
        velLimits(i)  = 0.25; // x ms^-1
    }
    joint_cddynamics->SetVelocityLimits(velLimits);

    rot_stiffness = 0;

    std::cout<< "finished init" << std::endl;
    return true;
}

void JointControllers::starting(const ros::Time& time)
{
    ROS_INFO("JointControllers::starting");
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_.q(i)    = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_.q(i)    = joint_msr_.q(i);
        q_target_(i)       = joint_msr_.q(i);

        pos_cmd_(i)        = joint_des_.q(i);
        tau_cmd_(i)        = 0;
        K_cmd(i)           = 0;
        D_cmd(i)           = 0;

        joint_handles_[i].setCommand(pos_cmd_(i));
        joint_handles_torque[i].setCommand(tau_cmd_(i));
        joint_handles_stiffness[i].setCommand(K_cmd(i));
        joint_handles_damping[i].setCommand(D_cmd(i));
    }

    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();

    ctrl_mode           = GRAV_COMP;
    cart_type           = VELOCITY_PASSIVE_DS;//VELOCITY_PASSIVE_DS;
    robot_ctrl_mode     = TORQUE_IMP;
    robot_ctrl_mode_tmp = robot_ctrl_mode;
    std::cout<< "finished starting" << std::endl;
}

void JointControllers::update(const ros::Time& time, const ros::Duration& period)
{
    // get measured joint positions and velocity
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_.q(i)           = joint_handles_[i].getPosition();
        joint_msr_.qdot(i)        = joint_handles_[i].getVelocity();
        qdot_msg.data[i]          = joint_msr_.qdot(i);
    }
    pub_qdot_.publish(qdot_msg);

    jnt_to_jac_solver_->JntToJac(joint_msr_.q,J_);
    fk_pos_solver_->JntToCart(joint_msr_.q, x_);

    double r,p,y;
    double x,z,w;
//    x_.M.GetRPY(r,p,y);
    x_.M.GetQuaternion(x,y,z,w);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_.p(0), x_.p(1), x_.p(2)) );
    tf::Quaternion q;
    q.setX(x);  q.setY(y);   q.setZ(z);   q.setW(w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));



   // ROS_INFO_STREAM_THROTTLE(0.5,"x_.M:          " << r << " " << p << " " << y);
    ROS_INFO_STREAM_THROTTLE(0.5,"x_.M:          " << x << " " << y << " " << z << " " << w);


    switch(ctrl_mode)
    {
    case CART_VELOCITIY:
    {
        if       (cart_type == VELOCITY_OPEN_LOOP)
        {
            velocity_open_loop_update(period);
        }else if (cart_type == VELOCITY_PASSIVE_DS){
            passive_ds_update();
        }

        robot_ctrl_mode = TORQUE_IMP;
        break;
    }
    case JOINT_POSITION:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"JOINT_POSITION");
        joint_cddynamics->SetDt(period.toSec());
        joint_cddynamics->SetTarget(q_target_.data);
        joint_cddynamics->Update();
        joint_cddynamics->GetState(joint_des_.q.data);

 //       joint_des_.q.data = joint_filt_.q.data;

        /*for(std::size_t i = 0; i < joint_handles_.size();i++){
            tau_cmd_(i)  =   -D_(i) * (joint_msr_.qdot(i)) - K_(i) * (joint_msr_.q(i)  - joint_filt_.q(i));
        }*/

        robot_ctrl_mode = POSITION_IMP;
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
        robot_ctrl_mode = TORQUE_IMP;
        break;
    }
    default:    // same as grav-comp
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"NONE");
        for(std::size_t i = 0; i < joint_handles_.size();i++){
            tau_cmd_(i)         = 0;
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
            robot_ctrl_mode     = TORQUE_IMP;
        }
        break;
    }
    }

      if(robot_ctrl_mode == TORQUE_IMP)
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"   TORQUE_IMP    ");
        for(size_t i=0; i<joint_handles_.size(); i++) {
            K_cmd(i)         = 0;
            D_cmd(i)         = 0;
            pos_cmd_(i)      = joint_des_.q(i);
            q_target_(i)     = joint_msr_.q(i);
        }
    }else{
        ROS_INFO_STREAM_THROTTLE(1.0,"   POSITION_IMP   ");
        for(size_t i=0; i<joint_handles_.size(); i++) {
            K_cmd(i)         = K_(i);
            D_cmd(i)         = D_(i);
            tau_cmd_(i)      = 0;
            pos_cmd_(i)      = joint_des_.q(i);
        }
    }

      ROS_INFO_STREAM_THROTTLE(0.5,"--------------");
      ROS_INFO_STREAM_THROTTLE(0.5,"K_cmd:    " << K_cmd(0) << " " << K_cmd(1) << " " << K_cmd(2) << " " << K_cmd(3) << " " << K_cmd(4) << " " << K_cmd(5) << " " << K_cmd(6));
      ROS_INFO_STREAM_THROTTLE(0.5,"D_cmd:    " << D_cmd(0) << " " << D_cmd(1) << " " << D_cmd(2) << " " << D_cmd(3) << " " << D_cmd(4) << " " << D_cmd(5) << " " << D_cmd(6));
      ROS_INFO_STREAM_THROTTLE(0.5,"tau_cmd_: " << tau_cmd_(0) << " " <<  tau_cmd_(1) << " " <<  tau_cmd_(2) << " " << tau_cmd_(3) << " " <<  tau_cmd_(4) << " " <<  tau_cmd_(5) << " " << tau_cmd_(6));
      ROS_INFO_STREAM_THROTTLE(0.5,"pos_cmd_: " << pos_cmd_(0) << " " <<  pos_cmd_(1) << " " <<  pos_cmd_(2) << " " << pos_cmd_(3) << " " <<  pos_cmd_(4) << " " <<  pos_cmd_(5) << " " << pos_cmd_(6));
      ROS_INFO_STREAM_THROTTLE(0.5,"--------------");

    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(pos_cmd_(i));
        joint_handles_torque[i].setCommand(tau_cmd_(i));
        joint_handles_stiffness[i].setCommand(K_cmd(i));
        joint_handles_damping[i].setCommand(D_cmd(i));
    }
}


void JointControllers::velocity_open_loop_update(const ros::Duration& period){
   // ROS_INFO_STREAM_THROTTLE(1.0,"CART_VELOCITIY");
   // ROS_INFO_STREAM_THROTTLE(1.0,"x_des_vel_: " << x_des_vel_.vel(0) << " "  << x_des_vel_.vel(1) << " "  << x_des_vel_.vel(2) << " " << x_des_vel_.rot(0) <<  " " << x_des_vel_.rot(1) << " "  << x_des_vel_.rot(2));
    ik_vel_solver_->CartToJnt(joint_msr_.q,x_des_vel_,joint_des_.qdot);
    for (std::size_t i = 0; i < joint_handles_.size(); i++){
        // integrating q_dot -> getting q (Euler method)
        joint_des_.q(i) = joint_des_.q(i) + period.toSec()*joint_des_.qdot(i);
        // joint position and velocity error to torque
        tau_cmd_(i)     =   -D_(i) * (joint_msr_.qdot(i) - joint_des_.qdot(i)) - K_(i) * (joint_msr_.q(i)  - joint_des_.q(i));

    }
}

void JointControllers::passive_ds_update(){
 //   ROS_INFO_STREAM_THROTTLE(1.0,"CART_VELOCITY_DS");

    x_orient_[0][0] = x_.M(0,0);
    x_orient_[0][1] = x_.M(0,1);
    x_orient_[0][2] = x_.M(0,2);

    x_orient_[1][0] = x_.M(1,0);
    x_orient_[1][1] = x_.M(1,1);
    x_orient_[1][2] = x_.M(1,2);

    x_orient_[2][0] = x_.M(2,0);
    x_orient_[2][1] = x_.M(2,1);
    x_orient_[2][2] = x_.M(2,2);

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

    x_des_orient_rot_.setRotation(x_des_orient_);


    err_orient = x_orient_ * x_des_orient_rot_.transpose();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_.p(0), x_.p(1), x_.p(2)) );
    tf::Quaternion q;
    ROS_INFO_STREAM_THROTTLE(0.5,"x_q: " << x_des_orient_.getX() << " " << x_des_orient_.getY() << " " << x_des_orient_.getZ() << " " << x_des_orient_.getW());
     transform.setRotation(x_des_orient_);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_des_orient_"));



    //	Vector3 err_orient_axis;
    //	realtype err_orient_angle;
    double err_orient_angle;
    err_orient.getRotation(q);

    double r_e,p_e,y_e;
    err_orient.getRPY(r_e,p_e,y_e);

    //	err_orient_axis = err_orient.GetNormRotationAxis();
    tf::Vector3 err_orient_axis = q.getAxis();
    //	err_orient_angle = err_orient.GetRotationAngle();
    err_orient_angle            = q.getAngle();
    if(std::isnan(err_orient_angle) || std::isinf(err_orient_angle)){
        ROS_WARN_STREAM_THROTTLE(0.5,"err_orient_angle: " << err_orient_angle );
        err_orient_angle = 0;
    }

    //	// rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    //	control_torque += err_orient_axis*err_orient_angle*(-rot_stiffness_);
    tf::Vector3 torque_orient = err_orient_axis * err_orient_angle * (-rot_stiffness);

    F_ee_des_(3) =  F_ee_des_(3) + torque_orient.getX();
    F_ee_des_(4) =  F_ee_des_(4) + torque_orient.getY();
    F_ee_des_(5) =  F_ee_des_(5) + torque_orient.getZ();

    ROS_INFO_STREAM_THROTTLE(0.5,"err_orient: " << r_e << " " << p_e << " " << y_e);
    ROS_INFO_STREAM_THROTTLE(0.5,"err_orient_angle: "<< err_orient_angle);

    //	// combine in one wrench vector
    //	Vector cartesian_control(6);
    //	cartesian_control.SetSubVector(0,control_force);
    //	cartesian_control.SetSubVector(3,control_torque);

    tau_cmd_.data = J_.data.transpose() * F_ee_des_;

}

void JointControllers::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO_THROTTLE(1.0,"command cart pos");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));
    //ctrl_type = CART_POSITION;
    //x_des_    = frame_des_;
    cmd_flag_ = true;
}

void JointControllers::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
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

void JointControllers::command_grav(const std_msgs::Bool& msg){
    ROS_INFO_THROTTLE(1.0,"command_grav");
    if(msg.data == true)
    {
        ROS_INFO("Starting Gravity compensation");
        for(std::size_t i = 0; i < joint_handles_.size();i++){
            K_tmp(i) = K_(i);
            D_tmp(i) = D_(i);
            K_(i)    = 0;
            D_(i)    = 0;
        }
        ctrl_mode = GRAV_COMP;
    }else{
        ROS_INFO("Stopping Gravity compensation");
        for(std::size_t i = 0; i < 7; i++){
            joint_des_.q(i)     = joint_msr_.q(i);
            joint_des_.qdot(i)  = 0;
            D_(i)               = D_tmp(i);
            K_(i)               = K_tmp(i);
        }
        joint_cddynamics->SetState(joint_msr_.q.data);
        ctrl_mode = JOINT_POSITION;
    }
}

void JointControllers::command_set_cart_type(const std_msgs::Int32& msg){

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

void JointControllers::command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg){
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
            q_target_(j)     = msg->data[j];
        }

        if(ctrl_mode == GRAV_COMP)
        {
            std_msgs::Bool bool_msg;
            bool_msg.data = false;
            command_grav(bool_msg);
        }

        joint_cddynamics->SetState(joint_msr_.q.data);
        ctrl_mode = JOINT_POSITION;
    }

}

void JointControllers::command_orient(const geometry_msgs::Quaternion &msg){
    x_des_orient_.setX(msg.x);
    x_des_orient_.setY(msg.y);
    x_des_orient_.setZ(msg.z);
    x_des_orient_.setW(msg.w);

}




void JointControllers::setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg){
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

void JointControllers::setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            D_(i)    = msg->data[i];
        }
    }else
    {
        ROS_INFO("Damping Num of Joint handles = %lu", joint_handles_.size());
    }
}

void JointControllers::damping_callback(damping_paramConfig &config, uint32_t level){
    D_(0) = config.damp_0_joint;
    D_(1) = config.damp_1_joint;
    D_(2) = config.damp_2_joint;
    D_(3) = config.damp_3_joint;
    D_(4) = config.damp_4_joint;
    D_(5) = config.damp_5_joint;
    D_(6) = config.damp_6_joint;
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        D_tmp(i) = D_(i);
    }
}

void JointControllers::stiffness_callback(lwr_controllers::stiffness_paramConfig& config, uint32_t level){
    K_(0) = config.K_0_joint;
    K_(1) = config.K_1_joint;
    K_(2) = config.K_2_joint;
    K_(3) = config.K_3_joint;
    K_(4) = config.K_4_joint;
    K_(5) = config.K_5_joint;
    K_(6) = config.K_6_joint;
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        K_tmp(i) = K_(i);
    }
}

void JointControllers::damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level){
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        D_(i)    = config.D;
        D_tmp(i) = D_(i);
    }

}

void JointControllers::stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level){
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        K_(i)    = config.K;
        K_tmp(i) = K_(i);
    }
}

void JointControllers::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
}

void JointControllers::rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level){
    rot_stiffness = config.rot_stiffness;
}


}                                                           // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointControllers, controller_interface::ControllerBase)
