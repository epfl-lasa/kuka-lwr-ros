#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

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

#include <dynamic_reconfigure/server.h>

#include <lwr_controllers/PIDConfig.h>
#include <lwr_controllers/stiffness_param_allConfig.h>
#include <lwr_controllers/damping_param_allConfig.h>



namespace lwr_controllers
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
    public:

        typedef enum Ctrl_type{
            POSITION,
            VELOCITIY
        }Ctrl_type;

	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& /*time*/);

        void command_pos(const geometry_msgs::PoseConstPtr& msg);
        void command_vel(const geometry_msgs::TwistConstPtr& msg);
        void setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg);

    private:

        void pid_callback(lwr_controllers::PIDConfig& config,uint32_t level);

        void damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level);

        void stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level);

	private:
        ros::Subscriber sub_command_pose_;
        ros::Subscriber sub_command_vel_;
        ros::Subscriber sub_stiff_;
        ros::Subscriber sub_damp_;
        tf::Transform transform;

        ros::Subscriber sub_gains_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose
        KDL::Frame P_err;	//position error for PID computation
        KDL::Frame I_err;	//position error for PID computation
        KDL::Frame D_err;	//position error for PID computation

        KDL::Vector x_v_des_; // desired linear velocity
        KDL::Twist  x_des_vel_;

		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

        KDL::JntArray K_, D_; // stiffness and damping



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

        /// Extra handles
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_damping;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_torque;
        std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_stiffness;

        /// Dynamic parameters

        ros::NodeHandle nd_pid, nd_K, nd_D;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig> >     dynamic_server_D_all_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig> >   dynamic_server_K_all_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::PIDConfig> >                   dynamic_server_PID_param;
        double Kp;
        double Kd;
        double Ki;

        Ctrl_type ctrl_type;
	};

}

#endif
