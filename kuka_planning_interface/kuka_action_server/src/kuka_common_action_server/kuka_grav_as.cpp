#include "kuka_common_action_server/kuka_grav_as.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace asrv{


Kuka_grav_as::Kuka_grav_as(ros::NodeHandle&  nh):
    Base_j_action(nh),
    Base_action_server(nh)
{
    des_j_stiffness.resize(KUKA_NUM_JOINTS);
    stiffness.resize(KUKA_NUM_JOINTS);
    first_stiffness.resize(KUKA_NUM_JOINTS);

    fri_sub =  nh.subscribe("/lwr/FRI_data",1,&Kuka_grav_as::fri_callback,this);

}

bool Kuka_grav_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    if(goal->JointStates.stiffness.size() != KUKA_NUM_JOINTS){
        ROS_ERROR("goal->JointStates.stiffness.size() != KUKA_NUM_JOINTS");
        return false;
    }

    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++)
    {
        des_j_stiffness(i) = goal->JointStates.stiffness[i];
    }

    // save initial stiffness before going to gravity compensation
    first_stiffness = stiffness;

    update_position(joint_sensed);
    ros::spinOnce();

    bool success = true;
    ros::Rate rate(100);
    while(ros::ok()) {
        update_position(joint_sensed);
        update_stiffness(des_j_stiffness);
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

    update_position(joint_sensed);
    update_stiffness(first_stiffness);
    ROS_INFO("Stopping GRAV_COMP");
    return success;
}

void Kuka_grav_as::fri_callback(const kuka_fri_bridge::FRI::ConstPtr& msg){

  /*  if(msg->damping.size() == damping.size()){
        for(std::size_t i = 0; i < damping.size();i++){
            damping(i) = msg->damping[i];
        }
    }*/
    if(msg->stiffness.size() == static_cast<std::size_t>(stiffness.size())){
        for(std::size_t i = 0; i < static_cast<std::size_t>(stiffness.size());i++){
            stiffness(i) = msg->stiffness[i];
        }
    }

}

}
