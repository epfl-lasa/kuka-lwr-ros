#ifndef LWR_CONTROLLERS__JOINT_KINEMATICS_IMP_H
#define LWR_CONTROLLERS__JOINT_KINEMATICS_IMP_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/PoseRPY.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <robot_motion_generation/CDDynamics.h>

#include <memory>

namespace lwr_controllers
{
    class JointKinematiscImp: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
    {
    public:

        typedef enum Ctrl_type{
            CART_POSITION,
            CART_VELOCITIY,
            JOINT_POSITION,
            GRAV_COMP
        }Ctrl_type;

    public:
        JointKinematiscImp();
        ~JointKinematiscImp();

        bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& /*time*/);

        void command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void command_cart_pos(const geometry_msgs::PoseConstPtr& msg);
        void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);
        void command_grav(const std_msgs::Bool& msg);

        void setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg);

    private:


        std::unique_ptr<motion::CDDynamics> joint_cddynamics;

        ros::Subscriber sub_command_grav_;
        ros::Subscriber sub_command_joint_pos_;
        ros::Subscriber sub_command_pose_;
        ros::Subscriber sub_command_vel_;
        ros::Subscriber sub_stiff_;
        ros::Subscriber sub_damp_;
        tf::Transform transform;

        ros::Subscriber sub_gains_;

        KDL::Frame x_;		//current pose
        KDL::Frame x_des_;	//desired pose

        KDL::Vector x_v_des_; // desired linear velocity
        KDL::Twist  x_des_vel_;

        KDL::Twist x_err_;

        KDL::JntArray q_cmd_; // computed set points
        KDL::JntArray q_msr_, q_des_,q_target_;


        KDL::Jacobian J_;	//Jacobian

        Eigen::MatrixXd J_pinv_;
        Eigen::Matrix<double,3,3> skew_;

        KDL::JntArray K_, D_, K_tmp; // stiffness and damping


        ros::Time           last_publish_time_;
        double              publish_rate_;


        struct quaternion_
        {
            KDL::Vector v;
            double a;
        } quat_curr_, quat_des_;

        KDL::Vector v_temp_;

        bool cmd_flag_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver>         jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>  fk_pos_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_pos_solver_;

        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_damping;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_stiffness;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_torque;

        Ctrl_type ctrl_type;
    };

}

#endif
