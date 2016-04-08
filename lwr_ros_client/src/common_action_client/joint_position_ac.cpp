#include "kuka_action_client/common_action_client/joint_position_ac.h"
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

namespace ac{

Joint_position_ac::Joint_position_ac(){



    goal.trajectory.joint_names.resize(7);
    for(std::size_t i = 0; i < 7;i++){
         goal.trajectory.joint_names[i] = "lwr_" + boost::lexical_cast<std::string>(i) + "_joint";
    }
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
   // joint_traj_msg.points.resize(1);
   // joint_traj_msg.points[0].positions.resize(7);
    traj_client_ = new TrajClient("/lwr/joint_trajectory_controller/follow_joint_trajectory/", true);
}
Joint_position_ac::~Joint_position_ac(){
    if(traj_client_!= NULL){
        delete traj_client_;traj_client_=NULL;
    }
}

void Joint_position_ac::wait_for_server(){
    ROS_INFO("Waiting for Joint Position Server to start");
    traj_client_->waitForServer();
    ROS_INFO("Joint Position Server STARTED");

    /*while(!traj_client_->waitForServer(ros::Duration(5.0))){
         ROS_INFO("Joint_position_ac Waiting for the joint_trajectory_action server");
     }*/
}


void Joint_position_ac::call_action(const std::vector<double>& joint_position_target){

    if(joint_position_target.size() == 7){
        for(std::size_t i = 0; i < 7;i++){
            goal.trajectory.points[0].positions[i] = joint_position_target[i];
            std::cout<< "pos: " << joint_position_target[i] << std::endl;
        }
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(2.0);
        traj_client_->sendGoal(goal);
        traj_client_->waitForResult();
        actionlib::SimpleClientGoalState state = getState();

        ROS_INFO("Action finished: %s",state.toString().c_str());

    }else{
        std::string msg = "Joint_position_ac::call_action joint_position.size() = " + boost::lexical_cast<std::string>(joint_position_target.size());
        ROS_ERROR(msg.c_str());
    }

}

//! Returns the current state of the action
actionlib::SimpleClientGoalState Joint_position_ac::getState()
{
  return traj_client_->getState();
}


}
