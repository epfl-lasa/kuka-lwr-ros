
#include <lwr_controllers/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <cmath>

namespace lwr_controllers
{

OneTaskInverseKinematics::OneTaskInverseKinematics() {}
OneTaskInverseKinematics::~OneTaskInverseKinematics() {}

bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
    if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
    {
        ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
        return false;
    }
    K_.resize(7);
    D_.resize(7);

    // $ rosrun rqt_reconfigure rqt_reconfigure
    nd_pid = ros::NodeHandle("PID_param");
    dynamic_server_PID_param.reset(new        dynamic_reconfigure::Server< lwr_controllers::PIDConfig   >(nd_pid));
    dynamic_server_PID_param->setCallback(     boost::bind(&OneTaskInverseKinematics::pid_callback, this, _1, _2));
    Kp = 16.0;
    Kd = 1.5;
    Ki = 0;

    nd_K   = ros::NodeHandle("K");
    nd_D   = ros::NodeHandle("D");

    dynamic_server_D_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig   >(nd_D));
    dynamic_server_K_all_param.reset(new    dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig >(nd_K));
    dynamic_server_D_all_param->setCallback( boost::bind(&OneTaskInverseKinematics::damping_all_callback,  this, _1, _2));
    dynamic_server_K_all_param->setCallback( boost::bind(&OneTaskInverseKinematics::stiffness_all_callback,this, _1, _2));


    // Get joint handles for all of the joints in the chain
    for(std::size_t i = 0; i < 7; i++)
    {
        joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()  + "_stiffness"));
        joint_handles_damping.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()    + "_damping"));
        joint_handles_torque.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()     + "_torque"));
    }

    ctrl_type   = POSITION;

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

    q_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    // get joint positions
    for(std::size_t i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i) = joint_handles_[i].getPosition();
        joint_msr_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_.q(i) = joint_msr_.q(i);
    }

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_.q, x_);

    //Desired posture is the current one
    x_des_ = x_;
    I_err.p.Zero();

    cmd_flag_ = false;

    sub_command_pose_ = nh_.subscribe("command_pos", 1, &OneTaskInverseKinematics::command_pos,     this);
    sub_command_vel_  = nh_.subscribe("command_vel", 1, &OneTaskInverseKinematics::command_vel,     this);
    sub_stiff_        = nh_.subscribe("stiffness",   1, &OneTaskInverseKinematics::setStiffness,    this);
    sub_damp_         = nh_.subscribe("damping",     1, &OneTaskInverseKinematics::setDamping,      this);
    publish_rate_     = 500;

    return true;
}

void OneTaskInverseKinematics::starting(const ros::Time& time)
{
    ROS_INFO("starting on one task inverse kinematics");
    cmd_flag_ = false;
    // get joint positions
    for(std::size_t  i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i) = joint_handles_[i].getPosition();
        joint_des_.q(i) =  joint_msr_.q(i);
        K_(i)                  = 1000.0;
        D_(i)                  = 0.7;
        joint_handles_[i].setCommand(joint_msr_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
        joint_handles_damping[i].setCommand(D_(i));

    }

    ctrl_type   = POSITION;
    // initialize time
    last_publish_time_ = time;

}

void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
{
    // ROS_INFO_THROTTLE(2.0," OneTaskInverseKinematics::update");
    // ROS_INFO_STREAM_THROTTLE(2.0,"period: " << period.toSec());

    // get joint positions
    for(std::size_t  i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i) = joint_handles_[i].getPosition();
    }

    if (cmd_flag_)
    {

        if(ctrl_type == VELOCITIY){
            ik_vel_solver_->CartToJnt(joint_msr_.q,x_des_vel_,joint_des_.qdot);
        }else{

            if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

                static tf::TransformBroadcaster br1, br2;
                x_des_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
                transform.setRotation(tf::Quaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a));
                transform.setOrigin(tf::Vector3(x_des_.p(0),x_des_.p(1),x_des_.p(2)));
                br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_des_"));

                x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
                transform.setRotation(tf::Quaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a));
                transform.setOrigin(tf::Vector3(x_.p(0),x_.p(1),x_.p(2)));
                br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));
            }

            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_.q, J_);

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_.q, x_);

            // end-effector position error
            KDL::Frame tmp;
            tmp.p = P_err.p;
            P_err.p = x_des_.p - x_.p;
            I_err.p = P_err.p + I_err.p;
            D_err.p = P_err.p - tmp.p;
            x_err_.vel = Kp * P_err.p + Ki*I_err.p + Kd * D_err.p;
//            x_err_.vel = Kp * P_err.p;

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

            if (Equal(x_, x_des_, 0.005) && (ctrl_type == POSITION))
            {
                ROS_INFO("On target");
                cmd_flag_ = false;
            }

        }

        // integrating q_dot -> getting q (Euler method)
        for (std::size_t  i = 0; i < joint_handles_.size(); i++)
            joint_des_.q(i) += period.toSec()*joint_des_.qdot(i);

        // joint limits saturation
        for (std::size_t  i =0;  i < joint_handles_.size(); i++)
        {
            if (joint_des_.q(i) < joint_limits_.min(i))
                joint_des_.q(i) = joint_limits_.min(i);
            if (joint_des_.q(i) > joint_limits_.max(i))
                joint_des_.q(i) = joint_limits_.max(i);
        }
    }

    // set controls for joints
    for (std::size_t  i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(joint_des_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
        joint_handles_damping[i].setCommand(D_(i));
    }

  /*
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
*/
}

void OneTaskInverseKinematics::stopping(const ros::Time& /*time*/){
    for(std::size_t  i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i) = joint_handles_[i].getPosition();
        joint_handles_[i].setCommand(joint_des_.q(i));
        joint_handles_stiffness[i].setCommand(K_(i));
        joint_handles_damping[i].setCommand(D_(i));
    }

    cmd_flag_ = false;
}

void OneTaskInverseKinematics::command_pos(const geometry_msgs::PoseConstPtr &msg)
{
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));
    ctrl_type = POSITION;
    x_des_    = frame_des_;
    cmd_flag_ = true;
}

void OneTaskInverseKinematics::command_vel(const geometry_msgs::TwistConstPtr &msg){

    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    ctrl_type = VELOCITIY;
    cmd_flag_ = true;

}

void OneTaskInverseKinematics::damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level){

   ROS_INFO_STREAM("----> D: " << config.D);
    for(std::size_t i = 0; i < joint_handles_damping.size();i++){
        D_(i)    = config.D;
    }
    ROS_INFO_STREAM("---->  D_(i) : " <<  D_(0) );



}

void OneTaskInverseKinematics::stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level){
    for(std::size_t i = 0; i < joint_handles_.size();i++){
        K_(i)    = config.K;
    }
}

void OneTaskInverseKinematics::setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            K_(i) = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Stiffness Num of Joint handles = %lu", joint_handles_.size());
    }
}

void OneTaskInverseKinematics::setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == joint_handles_.size()){
        for (std::size_t i = 0; i < joint_handles_.size(); ++i){
            D_(i) = msg->data[i];
        }
    }else
    {
        ROS_INFO("Damping Num of Joint handles = %lu", joint_handles_.size());
    }
}

void OneTaskInverseKinematics::pid_callback(lwr_controllers::PIDConfig& config,uint32_t level){
    Kp = config.Kp;
    Kd = config.Kd;
    Ki = config.Ki;
}



}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerBase)



