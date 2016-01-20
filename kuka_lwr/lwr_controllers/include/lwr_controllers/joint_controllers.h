
#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <lwr_controllers/damping_paramConfig.h>
#include <lwr_controllers/passive_ds_paramConfig.h>
#include <lwr_controllers/stiffness_paramConfig.h>
#include <lwr_controllers/stiffness_param_allConfig.h>
#include <lwr_controllers/damping_param_allConfig.h>
#include <lwr_controllers/rot_stiffnessConfig.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <robot_motion_generation/CDDynamics.h>

#include <boost/scoped_ptr.hpp>

#include <passive_ds_controller.h>

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace lwr_controllers
{

    class JointControllers: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
    public:

        typedef enum CTRL_MODE{
            CART_VELOCITIY,         /// velocity
            JOINT_POSITION,         /// standard joint position controller (for goto joint pose)
            GRAV_COMP               /// sets the controller into gravity compensation
        }CTRL_MODE;

        typedef enum CART_TYPE
        {
           VELOCITY_OPEN_LOOP=0,
           VELOCITY_PASSIVE_DS=1
        }CART_TYPE;

        typedef enum ROBOT_CTRL_MODE
        {
            POSITION_IMP,
            TORQUE_IMP
        } ROBOT_CTRL_MODE;


	public:

        JointControllers();
        ~JointControllers();

        bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);
    private:

        void velocity_open_loop_update(const ros::Duration& period);

        void passive_ds_update();

    private:

        void command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void command_cart_pos(const geometry_msgs::PoseConstPtr& msg);
        void command_cart_vel(const geometry_msgs::TwistConstPtr& msg);
        void command_grav(const std_msgs::Bool& msg);
        void command_set_cart_type(const std_msgs::Int32& msg);
        void command_orient(const geometry_msgs::Quaternion &msg);

        void setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg);

    private:

        void damping_callback(lwr_controllers::damping_paramConfig& config,uint32_t level);

        void stiffness_callback(lwr_controllers::stiffness_paramConfig& config, uint32_t level);

        void damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level);

        void stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level);

        void ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level);

        void rot_stiffness_callback(lwr_controllers::rot_stiffnessConfig& config,uint32_t level);

	private:

		ros::Subscriber sub_gains_;
		ros::Subscriber sub_posture_;

        ros::Subscriber sub_command_grav_;
        ros::Subscriber sub_command_joint_pos_;
        ros::Subscriber sub_command_pose_;
        ros::Subscriber sub_command_vel_;
        ros::Subscriber sub_command_orient_;
        ros::Subscriber sub_stiff_;
        ros::Subscriber sub_damp_;

        ros::Publisher pub_qdot_;
        std_msgs::Float64MultiArray qdot_msg;

        KDL::JntArray    tau_cmd_;
        KDL::JntArray    pos_cmd_;
        KDL::JntArray    q_target_;
        KDL::JntArray    K_, D_, K_tmp,D_tmp,K_cmd,D_cmd;
        KDL::JntArrayVel joint_filt_;

        tf::Matrix3x3    err_orient;


        std::size_t      num_ctrl_joints;

        KDL::Frame       x_;		//current pose
        tf::Matrix3x3    x_orient_;
        tf::Quaternion   x_des_orient_;
        tf::Matrix3x3    x_des_orient_rot_;


        double rot_stiffness;

        KDL::Twist      x_des_vel_;

        Eigen::VectorXd x_msr_;
        Vec             x_linear_des_;
        Vec             x_linear_msr_;
        Vec             F_linear_des_;     // desired linear force
        Eigen::VectorXd F_ee_des_;         // desired end-effector force

        KDL::Jacobian   J_;	//Jacobian

        Eigen::VectorXd xD;


        ros::Time           last_publish_time_;
        double              publish_rate_;


        struct quaternion_
        {
            KDL::Vector v;
            double a;
        } quat_curr_, quat_des_;

        KDL::Vector v_temp_;

        bool cmd_flag_;

        boost::scoped_ptr<KDL::ChainDynParam>               id_solver_gravity_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver>         jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>  fk_pos_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_pos_solver_;

        boost::scoped_ptr<motion::CDDynamics>                 joint_cddynamics;
        boost::scoped_ptr<DSController>                       passive_ds_controller;

        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::damping_paramConfig> >         dynamic_server_D_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig> >      dynamic_server_ds_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::stiffness_paramConfig> >       dynamic_server_K_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig> >     dynamic_server_D_all_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig> >   dynamic_server_K_all_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::rot_stiffnessConfig> >         dynamic_server_rot_stiffness_param;


        ros::NodeHandle nd1, nd2, nd3,nd4,nd5, nd6;

        CTRL_MODE       ctrl_mode;
        CART_TYPE       cart_type;
        ROBOT_CTRL_MODE robot_ctrl_mode, robot_ctrl_mode_tmp;

        /// Extra handles
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_damping;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_stiffness;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_torque;



	};

} // namespace

#endif
