#ifndef LWR_CONTROLLERS__KUKA_JOINT_STATE_CONTROLLER_H_
#define LWR_CONTROLLERS__KUKA_JOINT_STATE_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <control_toolbox/filters.h>

#include "KinematicChainControllerBase.h"


namespace lwr_controllers{

class KUKAJointStateController : public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface> {

public:

    KUKAJointStateController() : publish_rate_(0.0) {}

    virtual bool init(hardware_interface::JointStateInterface* hw,
                      ros::NodeHandle&                         root_nh,
                      ros::NodeHandle&                         controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time&);

private:
    std::vector<hardware_interface::JointStateHandle> joint_state_;

    // Publishers
    boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_pose_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > realtime_twist_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Accel> > realtime_accel_pub_;

    ros::Time last_publish_time_;
    double publish_rate_;
    unsigned int num_hw_joints_;
    void addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg);

    boost::shared_ptr<KDL::ChainFkSolverVel_recursive>  fk_vel_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    KDL::Frame          x_;     // current pos
    KDL::FrameVel          x_dot_, x_dot_prev_;     // current vel
    KDL::Wrench         x_dotdot_;     // current accel

    KDL::JntArray K_, D_;
    KDL::JntArrayVel joint_msr_states_;



};

}

#endif
