
#ifndef LWR_CONTROLLERS__JOINT_POSITION_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOIN_POSITION_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include <std_msgs/Float64MultiArray.h>
#include <boost/scoped_ptr.hpp>
#include <robot_motion_generation/CDDynamics.h>


namespace lwr_controllers
{

class JointPositionImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
{
public:


    JointPositionImpedanceController();
    ~JointPositionImpedanceController();

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time&);

    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void setStiffness(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void setDamping(const std_msgs::Float64MultiArray::ConstPtr &msg);


private:

    ros::Subscriber sub_stiff_;
    ros::Subscriber sub_damp_;
    ros::Subscriber sub_posture_;
    ros::Subscriber sub_action_;

    std::size_t num_ctrl_joints;

    KDL::JntArray q_msr_, q_des_,q_target_;
    KDL::JntArray K_, D_;

    motion::CDDynamics* cddynamics;

    std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_stiffness;



};


} // namespace

#endif

