#include "kuka_common_action_server/kuka_goto_joint_as.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace asrv{

Kuka_goto_joint_as::Kuka_goto_joint_as(ros::NodeHandle&  nh):
    Base_j_action(nh),
    Base_action_server(nh)
{
    joint_states_target.resize(KUKA_NUM_JOINTS);
    joint_states_diff.resize(KUKA_NUM_JOINTS);
    joint_states_des.resize(KUKA_NUM_JOINTS);

    model_dt = 100.0; // 100 Hz

    cddynamics = new motion::CDDynamics(KUKA_NUM_JOINTS,1.0/model_dt,1);
    motion::Vector velLimits(KUKA_NUM_JOINTS);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS; i++){
        velLimits(i) = 1; // ms^-1
    }
    cddynamics->SetVelocityLimits(velLimits);

}


bool Kuka_goto_joint_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){


    if(goal->JointStates.position.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("goal->JointStates.position.size() != KUKA_NUM_JOINTS)");
        return false;
    }
    activate_controller("joint_position_impedance_controller");


    ROS_INFO("Execution started");

    // Send the action target
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        joint_states_target(i) = goal->JointStates.position[i];
        joint_states_des(i)    = joint_states_target(i);
    }
    /*
        cddynamics->SetTarget(joint_states_target);
        cddynamics->SetState(joint_sensed);
        cddynamics->Update();
        cddynamics->GetState(joint_states_des);
     */

    update_position(joint_states_target);
    ros::spinOnce();

    bool    success = true;
    double  dist_joint    = (joint_sensed - joint_states_target).norm();

    ros::Rate rate(model_dt);
    while(ros::ok()){

        joint_states_diff = (joint_sensed - joint_states_target);
        dist_joint        = joint_states_diff.norm();

        //  cddynamics->SetTarget(joint_states_target);
        //  cddynamics->Update();
        //  cddynamics->GetState(joint_states_des);
        //  update_position(joint_states_des);

        ROS_INFO_THROTTLE(1.0,"current: %f %f %f %f %f %f %f",joint_sensed(0),joint_sensed(1),joint_sensed(2),joint_sensed(3),joint_sensed(4),joint_sensed(5),joint_sensed(6));
        ROS_INFO_THROTTLE(1.0,"target:  %f %f %f %f %f %f %f",joint_states_target(0),joint_states_target(1),joint_states_target(2),joint_states_target(3),joint_states_target(4),joint_states_target(5),joint_states_target(6));
        ROS_INFO_THROTTLE(1.0,"diff:    %f %f %f %f %f %f %f",joint_states_diff(0),joint_states_diff(1),joint_states_diff(2),joint_states_diff(3),joint_states_diff(4),joint_states_diff(5),joint_states_diff(6));
        ROS_INFO_THROTTLE(1.0,"error:   %f",dist_joint);

        if(dist_joint < 0.02){
            ROS_INFO("Desired joint position REACHED!");
            ROS_INFO("Terminated Joint Control Mode!");
            success  = true;
            break;
        }
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            success = false;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }


    std::cout<< "joint_sensed: " << joint_sensed << std::endl;

    ros::spinOnce();
    update_position(joint_sensed);
    ros::spinOnce();

    ROS_INFO("Stopping GOTO_JOINT");
    return success;

}

}
