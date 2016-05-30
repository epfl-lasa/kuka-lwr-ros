
#ifndef LWR_CONTROLLERS__JOINT_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <lwr_controllers/damping_paramConfig.h>
#include <lwr_controllers/passive_ds_paramConfig.h>
#include <lwr_controllers/stiffness_paramConfig.h>
#include <lwr_controllers/stiffness_param_allConfig.h>
#include <lwr_controllers/damping_param_allConfig.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>


#include <boost/scoped_ptr.hpp>

#include <realtime_tools/realtime_publisher.h>

#include "controllers/gravity_compensation.h"
#include "controllers/joint_position.h"
#include "controllers/cartesian_velocity.h"
#include "controllers/ff_fb_cartesian.h"
#include "controllers/cartesian_position.h"
#include "controllers/change_ctrl_mode.h"
#include "controllers/passive_ds.h"

#include "utils/definitions.h"
#include "utils/contact_safety.h"
#include "utils/speed_safety.h"
#include "utils/safety.h"


namespace lwr_controllers
{

    class JointControllers: public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
	{

	public:

        JointControllers();

        ~JointControllers();

        bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);

    private:

        void publish_open_loop_pos(const KDL::JntArray &q_des_, const ros::Duration& period, const ros::Time& time);

        void command_set_cart_type(const std_msgs::Int32& msg);
        void setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void command_string(const std_msgs::String::ConstPtr& msg);

        void command_safety_reset(const std_msgs::BoolConstPtr& msg);

    private:

        void damping_callback(lwr_controllers::damping_paramConfig& config,uint32_t level);

        void stiffness_callback(lwr_controllers::stiffness_paramConfig& config, uint32_t level);

        void damping_all_callback(lwr_controllers::damping_param_allConfig& config,uint32_t level);

        void stiffness_all_callback(lwr_controllers::stiffness_param_allConfig& config, uint32_t level);

	private:

        controllers::Change_ctrl_mode                          change_ctrl_mode;
        boost::scoped_ptr<controllers::FF_FB_cartesian>        ff_fb_controller;
        boost::scoped_ptr<controllers::Cartesian_velocity>     cartesian_velocity_controller;
        boost::scoped_ptr<controllers::Cartesian_position>     cartesian_position_controller;
        boost::scoped_ptr<controllers::Joint_position>         joint_position_controller;
        boost::scoped_ptr<controllers::Gravity_compensation>   gravity_compensation_controller;
        boost::scoped_ptr<controllers::Passive_ds>             passive_ds_controller;

		ros::Subscriber sub_gains_;
		ros::Subscriber sub_posture_;

        ros::Subscriber sub_stiff_;
        ros::Subscriber sub_damp_;

        ros::Publisher pub_qdot_,pub_F_,pub_tau_;
        std_msgs::Float64MultiArray qdot_msg, F_msg, tau_msg;

        KDL::JntArray       tau_cmd_;
        KDL::JntArray       pos_cmd_;
        KDL::JntArray       K_, D_,K_cmd,D_cmd, K_pos_, K_vel_;

        std::size_t         num_ctrl_joints;

        KDL::Frame          x_msr_;         // measured end-effector position
        KDL::FrameVel       x_dt_msr_;      // measured end-effector velocity
        KDL::Frame          x_des_;         // desired pose
        KDL::Jacobian       J_;             // Jacobian
        KDL::JntArrayVel    joint_vel_msr_;


        boost::scoped_ptr<KDL::ChainDynParam>               id_solver_gravity_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver>         jnt_to_jac_solver_;

        boost::shared_ptr<KDL::ChainFkSolverPos_recursive>  fk_pos_solver_;
        boost::shared_ptr<KDL::ChainFkSolverVel_recursive>  fk_vel_solver_;
        boost::shared_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
        boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_pos_solver_;

        /// Safety

        boost::shared_ptr<lwr::safety::Safety>          safety;
        boost::shared_ptr<lwr::safety::Contact_safety>  contact_safety;
        boost::shared_ptr<lwr::safety::Speed_safety>    speed_safety;

        /// Dynamic Parameters

        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::damping_paramConfig> >         dynamic_server_D_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::stiffness_paramConfig> >       dynamic_server_K_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::damping_param_allConfig> >     dynamic_server_D_all_param;
        boost::scoped_ptr< dynamic_reconfigure::Server< lwr_controllers::stiffness_param_allConfig> >   dynamic_server_K_all_param;

        ros::NodeHandle nd1, nd2, nd3,nd4;

        lwr_controllers::CTRL_MODE       ctrl_mode;
        lwr_controllers::ROBOT_CTRL_MODE robot_ctrl_mode;


        KDL::Frame                                                                          x_open_loop;
        std::string                                                                         frame_id;
        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> >   realtime_publisher;
        double                                                                              last_publish_time_;
        double                                                                              q_x,q_y,q_z,q_w; // Quaternion parameters
        double                                                                              publish_rate_;


	};

} // namespace

#endif
