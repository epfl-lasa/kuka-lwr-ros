#include "lwr_controllers/kuka_joint_state_controller.h"

namespace lwr_controllers
{

bool KUKAJointStateController::init(hardware_interface::JointStateInterface* robot, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
{

    KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, root_nh);

   // std::cout<< "before fk_pose_solver_";

    //std::cout<< "N segments: " << kdl_chain_.getNrOfSegments() << std::endl;

    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    K_.resize(7);
    D_.resize(7);

   // std::cout<< "#1" << std::endl;

    // get all joint names from the hardware interface
    const std::vector<std::string>& joint_names = robot->getNames();
    for (unsigned i=0; i<joint_names.size(); i++)
        ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    //std::cout<< "#2" << std::endl;

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
    }

    //std::cout<< "#3" << std::endl;

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

    // get joints and allocate message
    for (unsigned i=0; i<joint_names.size(); i++){
        joint_state_.push_back(robot->getHandle(joint_names[i]));
        realtime_pub_->msg_.name.push_back(joint_names[i]);
        realtime_pub_->msg_.position.push_back(0.0);
        realtime_pub_->msg_.velocity.push_back(0.0);
        realtime_pub_->msg_.effort.push_back(0.0);
    }

    //std::cout<< "#4" << std::endl;


    for (unsigned i=0; i<=7; i++){
          realtime_pub_->msg_.name.push_back("lwr_" + boost::lexical_cast<std::string>(i) + "_joint_stiffness");
          realtime_pub_->msg_.position.push_back(0.0);
          realtime_pub_->msg_.velocity.push_back(0.0);
          realtime_pub_->msg_.effort.push_back(0.0);
      }


    //std::cout<< "#5" << std::endl;

    realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(root_nh,"ee_pose",4));

    //std::cout<< "#6" << std::endl;

   // std::cout<< "joint_handles_.size(): " << joint_handles_.size() << std::endl;
   // std::cout<< "joint_msr_states_.q.size(): " << joint_msr_states_.q.columns() << std::endl;

    //joint_msr_states_.q.resize(joint_handles_.size());

    ROS_INFO_STREAM("joint_handles_.size(): " << joint_handles_.size());

    joint_msr_states_.q.resize(7);
    // get joint positions
    for(int i=0; i < 7; i++)
    {
        joint_msr_states_.q(i) = 0;
    }
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
    realtime_pose_pub_->msg_.position.x = x_.p.x();
    realtime_pose_pub_->msg_.position.y = x_.p.y();
    realtime_pose_pub_->msg_.position.z = x_.p.z();
    double qx,qy,qz,qw;
    x_.M.GetQuaternion(qx,qy,qz,qw);
    realtime_pose_pub_->msg_.orientation.x = qx;
    realtime_pose_pub_->msg_.orientation.y = qy;
    realtime_pose_pub_->msg_.orientation.z = qz;
    realtime_pose_pub_->msg_.orientation.w = qw;

    //std::cout<< "#6" << std::endl;


    // Get joint handles for all of the joints in the chain
  //  std::cout<< "push back handle stiffness" << std::endl;
 /*   for(std::size_t i = 0; i < 7; i++)
    {

        std::cout<< "kdl_chain_.segments[" << i << "].getJoint().getName(): " << kdl_chain_.segments[i].getJoint().getName() <<std::endl;
        //joint_handles_stiffness.push_back(robot->getHandle(kdl_chain_.segments[i].getJoint().getName()+ "_stiffness"));
    }*/
  /*  std::vector<std::string> names =   robot->getNames();
    for(std::size_t i = 0; i < names.size();i++){
       joint_handles_stiffness.push_back(robot->getHandle(("lwr_" + boost::lexical_cast<std::string>(i) + "_joint_stiffness")));
    }
*/
    return true;
}

void KUKAJointStateController::starting(const ros::Time& time)
{
    // initialize time
    last_publish_time_ = time;
}

void KUKAJointStateController::update(const ros::Time& time, const ros::Duration& period)
{
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

        // try to publish
        if (realtime_pub_->trylock()){
            // we're actually publishing, so increment time
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

            // populate joint state message
            realtime_pub_->msg_.header.stamp = time;
            for (unsigned i=0; i<joint_state_.size(); i++){
                realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
                realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
                realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
            }
            realtime_pub_->unlockAndPublish();

            for(int i=0; i < 7; i++)
            {
                joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            }
           /* ROS_INFO_STREAM_THROTTLE(2.0,"joint_msr_states_.q: " << joint_msr_states_.q(0) << " "
                                                                 << joint_msr_states_.q(1) << " "
                                                                 << joint_msr_states_.q(2) << " "
                                                                 << joint_msr_states_.q(3));*/
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

          /*  ROS_INFO_STREAM_THROTTLE(2.0,"joint_msr_states_.q.rows(): " << joint_msr_states_.q.rows() );
            ROS_INFO_STREAM_THROTTLE(2.0,"x_: " << x_.p(0) << " " << x_.p(1) << " " << x_.p(2) );

            ROS_INFO_STREAM_THROTTLE(2.0,"kdl_chain_.nrOfJoints:   " << kdl_chain_.getNrOfJoints());
            ROS_INFO_STREAM_THROTTLE(2.0,"kdl_chain_.nrOfSegments: " << kdl_chain_.getNrOfSegments());*/


            realtime_pose_pub_->msg_.position.x = x_.p.x();
            realtime_pose_pub_->msg_.position.y = x_.p.y();
            realtime_pose_pub_->msg_.position.z = x_.p.z();
            x_.M.GetQuaternion(realtime_pose_pub_->msg_.orientation.x,
                               realtime_pose_pub_->msg_.orientation.y,
                               realtime_pose_pub_->msg_.orientation.z,
                               realtime_pose_pub_->msg_.orientation.w);

            realtime_pose_pub_->unlockAndPublish();

          /*  for(std::size_t i = 0; i < 7;i++){
                K_(i) = joint_handles_stiffness[i].getPosition();
            }
            ROS_INFO_STREAM_THROTTLE(2.0,"K: " << K_(0));*/

        }
    }
}

void KUKAJointStateController::stopping(const ros::Time& /*time*/){

}

}


PLUGINLIB_EXPORT_CLASS( lwr_controllers::KUKAJointStateController, controller_interface::ControllerBase)
