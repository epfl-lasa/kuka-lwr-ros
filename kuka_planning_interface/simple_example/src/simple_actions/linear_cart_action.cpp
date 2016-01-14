#include "simple_actions/linear_cart_action.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>

namespace simple_actions {



Linear_cart_action::Linear_cart_action(ros::NodeHandle& nh):
    Base_ee_action(nh),
    Base_action_server(nh)
{

    linear_cddynamics   = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,4) );
    angular_cddynamics  = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,1) );

    motion::Vector velLimits(3);
    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 1; // x ms^-1
    }
    linear_cddynamics->SetVelocityLimits(velLimits);

    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 0.02; // x ms^-1
    }
    angular_cddynamics->SetVelocityLimits(velLimits);


}

bool Linear_cart_action::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){

    if(!activate_controller("one_task_inverse_kinematics")){
        return false;
    }

    ros::spinOnce();

    // wait until first topic has been received
    b_received = false;
    while(b_received)
    {
        sleep(1);
    }
    sleep(1);

    tf::Vector3     current_origin  = ee_pose_current.getOrigin();
    tf::Quaternion  current_orient  = ee_pose_current.getRotation();

    static tf::TransformBroadcaster br1;
    tf::Transform transform;

    transform.setOrigin(current_origin);
    transform.setRotation(current_orient);
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));




    first_origin        = current_origin;
    target_p1           = first_origin + tf::Vector3(0,0.1,0);
    target_p2           = first_origin - tf::Vector3(0,0.1,0);
    tf::Matrix3x3 tmp1,tmp2;
    double roll, pitch, yaw;


    tmp2.setRPY(M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p1.setRPY(roll,pitch,yaw);

    tmp2.setRPY(-M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p2.setRPY(roll,pitch,yaw);

    target_origin       = target_p1;
    target_orientation  = target_R_p1;

    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    double control_rate = 100;
    ros::Rate loop_rate(control_rate);
    bool success = true;

    static tf::TransformBroadcaster br;

    motion::Vector filter_vel(3);
    filter_vel.setZero();

    linear_cddynamics->SetState(filter_vel);
    linear_cddynamics->SetDt(1.0/control_rate);

    angular_cddynamics->SetState(filter_vel);
    angular_cddynamics->SetDt(1.0/control_rate);


    // use position or velocity interface
    bool b_position = false;


    ROS_INFO("starting Linear Action");
    while(ros::ok()) {

        current_origin = ee_pose_current.getOrigin();
        current_orient = ee_pose_current.getRotation();

        transform.setOrigin(target_origin);
        transform.setRotation(target_orientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));


        transform.setOrigin(current_origin);
        transform.setRotation(current_orient);
        br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

        simple_line_policy(linear_velocity,angular_velocity,current_origin,current_orient,control_rate);

        /// Filter linear velocity
        filter_vel(0) = linear_velocity(0);
        filter_vel(1) = linear_velocity(1);
        filter_vel(2) = linear_velocity(2);

        linear_cddynamics->SetTarget(filter_vel);
        linear_cddynamics->Update();
        linear_cddynamics->GetState(filter_vel);

        ee_vel_msg.linear.x  = filter_vel(0);
        ee_vel_msg.linear.y  = filter_vel(1);
        ee_vel_msg.linear.z  = filter_vel(2);

        /// Filter angular velocity
        filter_vel(0) = angular_velocity(0);
        filter_vel(1) = angular_velocity(1);
        filter_vel(2) = angular_velocity(2);

        angular_cddynamics->SetTarget(filter_vel);
        angular_cddynamics->Update();
        angular_cddynamics->GetState(filter_vel);

        ee_vel_msg.angular.x = angular_velocity(0);
        ee_vel_msg.angular.y = angular_velocity(1);
        ee_vel_msg.angular.z = angular_velocity(2);


      //  ee_vel_msg.linear.x  = 0;
       // ee_vel_msg.linear.y  = 0;
       // ee_vel_msg.linear.z  = 0;
      //  ee_vel_msg.angular.x = 0;
      //  ee_vel_msg.angular.y = 0;
      //  ee_vel_msg.angular.z = 0;

        ee_pos_msg.position.x    = target_origin.x();
        ee_pos_msg.position.y    = target_origin.y();
        ee_pos_msg.position.z    = target_origin.z();
        ee_pos_msg.orientation.x = target_orientation.x();
        ee_pos_msg.orientation.y = target_orientation.y();
        ee_pos_msg.orientation.z = target_orientation.z();
        ee_pos_msg.orientation.w = target_orientation.w();

        if(b_position)
        {
           sendPose(ee_pos_msg);
        }else{
           sendVel(ee_vel_msg);
        }

        feedback.progress = 0;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            success = false;
            break;
        }

         ros::spinOnce();
         loop_rate.sleep();
    }


    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendVel(ee_vel_msg);

    return success;

}


void Linear_cart_action::simple_line_policy(Eigen::Vector3d& linear_velocity,
                                            Eigen::Vector3d& angular_velocity,
                                            const tf::Vector3 &current_origin,
                                            const tf::Quaternion &current_orient,
                                            double rate)
{

   tf::Vector3 velocity = (target_origin - current_origin);
              velocity  = (velocity.normalize()) * 0.05; // 0.05 ms^-1

     linear_velocity(0) = velocity.x();
     linear_velocity(1) = velocity.y();
     linear_velocity(2) = velocity.z();

     tf::Quaternion qdiff =  target_orientation - current_orient;
     Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
     Eigen::Quaternion<double>   q(current_orient.getW(),current_orient.getX(),current_orient.getY(), current_orient.getZ());

     angular_velocity   = motion::d2qw<double>(q,dq);

   //  ROS_INFO_STREAM_THROTTLE(1.0,"current:  " << current_origin.x() << " " << current_origin.y() << " " << current_origin.z() );
   //  ROS_INFO_STREAM_THROTTLE(1.0,"target:   " << target_origin.x() << " " << target_origin.y() << " " << target_origin.z() );
     ROS_INFO_STREAM_THROTTLE(1.0,"distance: " << (current_origin - target_origin).length() );

     if((current_origin - target_p1).length() < 0.005)
     {
         target_origin      = target_p2;
         target_orientation = target_R_p2;
     }

     if((current_origin - target_p2).length() < 0.005)
     {
         target_origin      = target_p1;
         target_orientation = target_R_p1;
     }

}

}
