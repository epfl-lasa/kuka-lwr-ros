#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_position_impedance_controller.h>


namespace lwr_controllers
{

JointPositionImpedanceController::JointPositionImpedanceController(){}

JointPositionImpedanceController::~JointPositionImpedanceController(){
    if(cddynamics != NULL){
        delete cddynamics; cddynamics=NULL;
    }
}

bool JointPositionImpedanceController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n){

    KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n);

    ROS_INFO("finished joint position impedance init");

    num_ctrl_joints = 7;

    // Get joint handles for all of the joints in the chain
    for(std::size_t i = 0; i < num_ctrl_joints; i++)
    {
        joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_stiffness"));
    }


    K_.resize(num_ctrl_joints);
    D_.resize(num_ctrl_joints);

    for (std::size_t i = 0; i < joint_handles_.size(); ++i){
        if ( !n.getParam("stiffness_gains", K_(i) ) ){
            ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (std::size_t i = 0; i < joint_handles_.size(); ++i){
        if ( !n.getParam("damping_gains", D_(i)) ){
            ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    q_msr_.resize(num_ctrl_joints);
    q_msr_.data.setZero();
    q_des_.resize(num_ctrl_joints);
    q_des_.data.setZero();
    q_target_.resize(num_ctrl_joints);
    q_target_.data.setZero();

    sub_stiff_   = nh_.subscribe("stiffness", 1, &JointPositionImpedanceController::setStiffness, this);
    sub_damp_    = nh_.subscribe("damping", 1, &JointPositionImpedanceController::setDamping, this);
    sub_posture_ = nh_.subscribe("command", 1, &JointPositionImpedanceController::command, this);

    cddynamics = new motion::CDDynamics(num_ctrl_joints,1e-6,1);

    motion::Vector velLimits(num_ctrl_joints);
    for(std::size_t i = 0; i < num_ctrl_joints; i++){
        velLimits(i)  = 1; // x ms^-1
    }
    cddynamics->SetVelocityLimits(velLimits);

    return true;

}

void JointPositionImpedanceController::starting(const ros::Time& time){

    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
        q_msr_(i)         = joint_handles_[i].getPosition();
        q_target_(i)      = q_msr_(i);
        q_des_(i)         = q_msr_(i);

        joint_handles_[i].setCommand(q_des_(i));
        joint_handles_stiffness[i].setCommand(K_(i));
    }
    cddynamics->SetState(q_msr_.data);

}

void JointPositionImpedanceController::update(const ros::Time& time, const ros::Duration& period){

  /*  std::string tmp = "update q_des: " + boost::lexical_cast<std::string>(q_des_(0)) + " " +
                                  boost::lexical_cast<std::string>(q_des_(1)) + " " +
                                  boost::lexical_cast<std::string>(q_des_(2));

    ROS_INFO_THROTTLE(4.0,tmp.c_str());*/

   // ROS_INFO_STREAM_THROTTLE(2.0,"period: " << period.toSec());

    cddynamics->SetDt(period.toSec());
    cddynamics->SetTarget(q_target_.data);
    cddynamics->Update();
    cddynamics->GetState(q_des_.data);

    for(size_t i=0; i<joint_handles_.size(); i++) {
        q_msr_(i)         =  joint_handles_[i].getPosition();
        joint_handles_[i].setCommand(q_des_(i));
        joint_handles_stiffness[i].setCommand(K_(i));
    }

}

void JointPositionImpedanceController::stopping(const ros::Time&){
    for(size_t i=0; i<joint_handles_.size(); i++) {
        q_msr_(i)         =  joint_handles_[i].getPosition();
        joint_handles_[i].setCommand(q_msr_(i));
        joint_handles_stiffness[i].setCommand(K_(i));
    }
}

void JointPositionImpedanceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {//   ROS_INFO("Joint impedance setting desired position");
        for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            q_target_(j) = msg->data[j];
    }

    //cddynamics->SetState();
   // cddynamics->SetTarget(q_target_.data);
}

void JointPositionImpedanceController::setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            K_(i) = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Stiffness Num of Joint handles = %lu", joint_handles_.size());
    }
/*
    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    ROS_INFO("New gains K: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
             K_(0), K_(1), K_(2), K_(3), K_(4), K_(5), K_(6));
    ROS_INFO("New gains D: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
             D_(0), D_(1), D_(2), D_(3), D_(4), D_(5), D_(6));*/

}

void JointPositionImpedanceController::setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            D_(i) = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Damping Num of Joint handles = %lu", joint_handles_.size());
    }
}



}

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointPositionImpedanceController, controller_interface::ControllerBase)

