#include "lwr_controllers/kuka_joint_state_controller.h"
#include <tf/transform_broadcaster.h>

namespace lwr_controllers
{

bool KUKAJointStateController::init(hardware_interface::JointStateInterface* robot, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
{

    std::cout<<"   "<<std::endl;
    ROS_INFO("init KUKAJointStateController");
    std::cout<<"   "<<std::endl;
    if( !(KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, controller_nh)) )
    {
        ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
        return false;
    }


    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    K_.resize(7);
    D_.resize(7);

    // std::cout<< "#1" << std::endl;

    // get all joint names from the hardware interface
    const std::vector<std::string>& joint_names = robot->getNames();
    for (unsigned i=0; i<joint_names.size(); i++)
        ROS_DEBUG("Got joint %s", joint_names[i].c_str());

   // std::cout<< "#2" << std::endl;

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

    for (unsigned i=0; i<=7; i++){
          realtime_pub_->msg_.name.push_back("lwr_" + boost::lexical_cast<std::string>(i) + "_joint_stiffness");
          realtime_pub_->msg_.position.push_back(0.0);
          realtime_pub_->msg_.velocity.push_back(0.0);
          realtime_pub_->msg_.effort.push_back(0.0);
      }

    for (unsigned i=0; i<=7; i++){
          realtime_pub_->msg_.name.push_back("lwr_" + boost::lexical_cast<std::string>(i) + "_joint_damping");
          realtime_pub_->msg_.position.push_back(0.0);
          realtime_pub_->msg_.velocity.push_back(0.0);
          realtime_pub_->msg_.effort.push_back(0.0);
      }

    for (unsigned i=0; i<=7; i++){
          realtime_pub_->msg_.name.push_back("lwr_" + boost::lexical_cast<std::string>(i) + "_joint_torque");
          realtime_pub_->msg_.position.push_back(0.0);
          realtime_pub_->msg_.velocity.push_back(0.0);
          realtime_pub_->msg_.effort.push_back(0.0);
      }

    // ee pos/vel/acc publishers
    realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(root_nh,"ee_pose",4));
    realtime_twist_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(root_nh,"ee_vel",4));
    realtime_accel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Accel>(root_nh,"ee_accel",4));

    std::string name_space = nh_.getNamespace();


    std::string tip_name;
    if (!nh_.getParam(name_space + "/tip_name", tip_name))
    {
        ROS_ERROR_STREAM("kuka_joint_state_controller: No tip name found on parameter server ("<<nh_.getNamespace()<<"/tip_name)");
        return false;
    }

    //std::cout<< "#6" << std::endl;

   // std::cout<< "joint_handles_.size(): " << joint_handles_.size() << std::endl;
   // std::cout<< "joint_msr_.q.size(): " << joint_msr_.q.columns() << std::endl;

    ROS_INFO_STREAM("joint_handles_.size(): " << joint_handles_.size());

    joint_msr_states_.q.resize(7);
    joint_msr_states_.qdot.resize(7);

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
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {

        // try to publish
        if (realtime_pub_->trylock()) {
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
        }


        /** CARTESIAN pos/vel/accel computation **/

        for(size_t i=0; i<7; i++) {
            joint_msr_states_.q(i)           = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i)        = joint_handles_[i].getVelocity();
           // qdot_msg.data[i]          = joint_msr_.qdot(i);
        }
        /* ROS_INFO_STREAM_THROTTLE(2.0,"joint_msr_.q: " << joint_msr_.q(0) << " "
                                                              << joint_msr_.q(1) << " "
                                                              << joint_msr_.q(2) << " "
                                                              << joint_msr_.q(3));*/
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        fk_vel_solver_->JntToCart(joint_msr_states_,x_dot_);
        x_dot_prev_ = x_dot_;

        // Compute x_dotdot
        for (size_t j=0; j<3; j++) {
            x_dotdot_.force(j) = filters::exponentialSmoothing((x_dot_.GetTwist().vel(j)-x_dot_prev_.GetTwist().vel(j))/period.toSec(), x_dotdot_.force(j), 0.2);
            x_dotdot_.torque(j) = filters::exponentialSmoothing((x_dot_.GetTwist().rot(j)-x_dot_prev_.GetTwist().rot(j))/period.toSec(), x_dotdot_.torque(j), 0.2);
        }


        /* ROS_INFO_STREAM_THROTTLE(2.0,"x_: " << x_.p(0) << " " << x_.p(1) << " " << x_.p(2) );
        ROS_INFO_STREAM_THROTTLE(2.0,"kdl_chain_.nrOfJoints:   " << kdl_chain_.getNrOfJoints());
        ROS_INFO_STREAM_THROTTLE(2.0,"kdl_chain_.nrOfSegments: " << kdl_chain_.getNrOfSegments());*/



//        static tf::TransformBroadcaster br1;
//        tf::Transform transform;
//        double x,y,z,w;
//        x_.M.GetQuaternion(x,y,z,w);
//        transform.setRotation(tf::Quaternion(x,y,z,w));
//        transform.setOrigin(tf::Vector3(x_.p.x(),x_.p.y(),x_.p.z()));
//        br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "EOF"));

        // Publish pos/vel/acc
        if (realtime_pose_pub_->trylock()) {
            // populate ee cartesian pose message
            realtime_pose_pub_->msg_.position.x = x_.p(0);
            realtime_pose_pub_->msg_.position.y = x_.p(1);
            realtime_pose_pub_->msg_.position.z = x_.p(2);

            x_.M.GetQuaternion(realtime_pose_pub_->msg_.orientation.x,
                                   realtime_pose_pub_->msg_.orientation.y,
                                   realtime_pose_pub_->msg_.orientation.z,
                                   realtime_pose_pub_->msg_.orientation.w);

            realtime_pose_pub_->unlockAndPublish();
        }

        if (realtime_twist_pub_->trylock()){
            // populate vel message
            realtime_twist_pub_->msg_.linear.x = x_dot_.GetTwist().vel(0);
            realtime_twist_pub_->msg_.linear.y = x_dot_.GetTwist().vel(1);
            realtime_twist_pub_->msg_.linear.z = x_dot_.GetTwist().vel(2);

            realtime_twist_pub_->msg_.angular.x = x_dot_.GetTwist().rot(0);
            realtime_twist_pub_->msg_.angular.y = x_dot_.GetTwist().rot(1);
            realtime_twist_pub_->msg_.angular.z = x_dot_.GetTwist().rot(2);

            realtime_twist_pub_->unlockAndPublish();
        }

        if (realtime_accel_pub_->trylock()){
            // populate accel message
            realtime_accel_pub_->msg_.linear.x = x_dotdot_.force(0);
            realtime_accel_pub_->msg_.linear.y = x_dotdot_.force(1);
            realtime_accel_pub_->msg_.linear.z = x_dotdot_.force(2);

            realtime_accel_pub_->msg_.angular.x = x_dotdot_.torque(0);
            realtime_accel_pub_->msg_.angular.y = x_dotdot_.torque(1);
            realtime_accel_pub_->msg_.angular.z = x_dotdot_.torque(2);

            realtime_accel_pub_->unlockAndPublish();
        }
    }
}

void KUKAJointStateController::stopping(const ros::Time& /*time*/){

}

}


PLUGINLIB_EXPORT_CLASS( lwr_controllers::KUKAJointStateController, controller_interface::ControllerBase)
