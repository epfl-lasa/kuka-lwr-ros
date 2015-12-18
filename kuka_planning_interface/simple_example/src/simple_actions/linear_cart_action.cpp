#include "simple_actions/linear_cart_action.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>

namespace simple_actions {



Linear_cart_action::Linear_cart_action(ros::NodeHandle& nh):
    Base_ee_action(nh),
    Base_action_server(nh)
{

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
    tf::Transform transform;

    ROS_INFO("starting Linear Action");
    while(ros::ok()) {

        current_origin = ee_pose_current.getOrigin();
        current_orient = ee_pose_current.getRotation();

        transform.setOrigin(target_origin);
        transform.setRotation(target_orientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));

        simple_line_policy(linear_velocity,angular_velocity,current_origin,current_orient,control_rate);

        ee_vel_msg.linear.x  = linear_velocity(0);
        ee_vel_msg.linear.y  = linear_velocity(1);
        ee_vel_msg.linear.z  = linear_velocity(2);
        ee_vel_msg.angular.x = angular_velocity(0);
        ee_vel_msg.angular.y = angular_velocity(1);
        ee_vel_msg.angular.z = angular_velocity(2);

        sendVel(ee_vel_msg);


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

     ROS_INFO_STREAM_THROTTLE(1.0,"current:  " << current_origin.x() << " " << current_origin.y() << " " << current_origin.z() );
     ROS_INFO_STREAM_THROTTLE(1.0,"target:   " << target_origin.x() << " " << target_origin.y() << " " << target_origin.z() );
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
