#include "controllers/cartesian_velocity.h"

namespace controllers{

Cartesian_velocity::Cartesian_velocity(ros::NodeHandle& nh,
                                       Change_ctrl_mode& change_ctrl_mode,
                                       boost::shared_ptr<KDL::ChainIkSolverVel_pinv>& ik_vel_solver,
                                       boost::shared_ptr<KDL::ChainFkSolverPos_recursive>& fk_pos_solver,
                                       double publish_rate,
                                       const std::string& frame_id)
    : Base_controllers(lwr_controllers::CTRL_MODE::CART_VELOCITIY),
      change_ctrl_mode(change_ctrl_mode),
      ik_vel_solver_(ik_vel_solver),
      fk_pos_solver(fk_pos_solver),
      publish_rate_(publish_rate),
      frame_id(frame_id)
{


    realtime_publisher.reset( new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(nh,"x_open_loop",1) );

    sub_command_vel_       = nh.subscribe("command_vel",      1, &Cartesian_velocity::command_cart_vel,     this);
    sub_command_orient_    = nh.subscribe("command_orient",   1 ,&Cartesian_velocity::command_orient,       this);

    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    bFirst = false;
}

void Cartesian_velocity::stop(){
    ROS_INFO_STREAM("stopping [CARTESIAN VELOCITY]");
    x_des_vel_.vel.Zero();
    x_des_vel_.rot.Zero();
    bFirst = false;
}


void Cartesian_velocity::cart_vel_update(KDL::JntArray&             tau_cmd,
                                         KDL::JntArrayAcc&          joint_des,
                                         const KDL::JntArrayAcc&    q_msr,
                                         const KDL::JntArray&       K,
                                         const KDL::JntArray&       D,
                                         const ros::Duration&       period,
                                         const ros::Time&           time)
{

    ik_vel_solver_->CartToJnt(q_msr.q,x_des_vel_,joint_des.qdot);
    for (int i = 0; i < joint_des.q.data.size(); i++){
        // integrating q_dot -> getting q (Euler method)
        joint_des.q(i) = joint_des.q(i) + period.toSec()*joint_des.qdot(i);
        // joint position and velocity error to torque
        tau_cmd(i)     =   -D(i) * (q_msr.qdot(i) - joint_des.qdot(i)) - K(i) * (q_msr.q(i)  - joint_des.q(i));
    }

    fk_pos_solver->JntToCart(joint_des.q,x_open_loop);
    publish_open_loop_pos(period,time);

}

void Cartesian_velocity::publish_open_loop_pos(const ros::Duration& period, const ros::Time& time){

        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + period.toSec();

        if(last_publish_time_ >= 1.0/publish_rate_){
            if (realtime_publisher->trylock()){
                realtime_publisher->msg_.header.frame_id     = frame_id;
                realtime_publisher->msg_.header.stamp        = time;
                realtime_publisher->msg_.pose.position.x     = x_open_loop.p.x();
                realtime_publisher->msg_.pose.position.y     = x_open_loop.p.y();
                realtime_publisher->msg_.pose.position.z     = x_open_loop.p.z();

                x_open_loop.M.GetQuaternion(q_x,q_y,q_z,q_w);

                realtime_publisher->msg_.pose.orientation.x  = q_x;
                realtime_publisher->msg_.pose.orientation.y  = q_y;
                realtime_publisher->msg_.pose.orientation.z  = q_z;
                realtime_publisher->msg_.pose.orientation.w  = q_w;
                realtime_publisher->unlockAndPublish();
                last_publish_time_ = 0;
            }
        }
}

void Cartesian_velocity::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    ROS_INFO_THROTTLE(1.0,"command_cart_vel");
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;
    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_VELOCITIY);
    }
    bFirst            = true;
}

void Cartesian_velocity::command_orient(const geometry_msgs::Quaternion &msg){
    x_des_orient_.setX(msg.x);
    x_des_orient_.setY(msg.y);
    x_des_orient_.setZ(msg.z);
    x_des_orient_.setW(msg.w);
}





}
