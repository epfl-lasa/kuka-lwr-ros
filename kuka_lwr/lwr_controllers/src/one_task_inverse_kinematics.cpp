
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

        ctrl_type   = POSITION;

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //Desired posture is the current one
        x_des_ = x_;

        cmd_flag_ = false;

        sub_command_pose_ = nh_.subscribe("command_pos", 1, &OneTaskInverseKinematics::command_pos, this);
        sub_command_vel_  = nh_.subscribe("command_vel", 1, &OneTaskInverseKinematics::command_vel, this);

        return true;
    }

    void OneTaskInverseKinematics::starting(const ros::Time& time)
    {
        ROS_INFO("starting on one task inverse kinematics");
        cmd_flag_ = false;
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_des_states_.q(i) =  joint_msr_states_.q(i);
        }
        ctrl_type   = POSITION;

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

    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {

        ROS_INFO_THROTTLE(2.0," OneTaskInverseKinematics::update");
        ROS_INFO_STREAM_THROTTLE(2.0,"period: " << period.toSec());


        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

       /* ROS_INFO_STREAM_THROTTLE(1.0,"joint: " << joint_des_states_.q(0) << " " <<
                                                  joint_des_states_.q(1) << " " <<
                                                  joint_des_states_.q(2) << " " <<
                                                  joint_des_states_.q(3) << " " <<
                                                  joint_des_states_.q(4) << " " <<
                                                  joint_des_states_.q(5));*/

        static tf::TransformBroadcaster br1, br2;
        x_des_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
        transform.setRotation(tf::Quaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a));
        transform.setOrigin(tf::Vector3(x_des_.p(0),x_des_.p(1),x_des_.p(2)));
        br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_des_"));

        x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
        transform.setRotation(tf::Quaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a));
        transform.setOrigin(tf::Vector3(x_.p(0),x_.p(1),x_.p(2)));
        br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

        if (cmd_flag_)
        {

            ROS_INFO_THROTTLE(2.0," OneTaskInverseKinematics::update cmd_flag_");


            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

            if(ctrl_type == POSITION)
            {
                ROS_INFO_THROTTLE(2.0,"POSITION");
            }else if(ctrl_type == VELOCITIY){
                ROS_INFO_THROTTLE(2.0,"VELOCITIY");
            }else{
                ROS_INFO_THROTTLE(2.0,"UNKNOWN");
            }


            if(ctrl_type == VELOCITIY){
                ROS_INFO_THROTTLE(2.0,"HERE");
                x_des_.M = KDL::Rotation::RPY(0,-M_PI/2,0);
                x_des_.p = x_.p + x_v_des_ * 0.01;
                ROS_INFO_STREAM_THROTTLE(2.0,"x_v_des_   : " << x_v_des_(0) << " " << x_v_des_(1) << " " << x_v_des_(2));
                ROS_INFO_STREAM_THROTTLE(2.0,"x_.p       : " << x_.p(0) << " " << x_.p(1) << " " << x_.p(2));
                ROS_INFO_STREAM_THROTTLE(2.0,"x_des.p    : " << x_des_.p(0) << " " << x_des_.p(1) << " " << x_des_.p(2));
            }


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
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            if (Equal(x_, x_des_, 0.005) && (ctrl_type == POSITION))
            {
                ROS_INFO("On target");
                cmd_flag_ = false;
            }
        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

    }

    void OneTaskInverseKinematics::stopping(const ros::Time& /*time*/){
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

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

       // KDL::Vector linear(,msg->linear.y,msg->linear.z);
       // KDL::Rotation rot = KDL::Rotation::RPY(msg->angular.x,msg->angular.y,msg->angular.z);
       // ROS_INFO_THROTTLE(2.0,"command_vel");

        x_v_des_.x(msg->linear.x);
        x_v_des_.y(msg->linear.y);
        x_v_des_.z(msg->linear.z);

       // x_des_.p = x_.p + linear;
      //  x_des_.M = x_.M * rot;

        ctrl_type = VELOCITIY;
        cmd_flag_ = true;

    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerBase)
