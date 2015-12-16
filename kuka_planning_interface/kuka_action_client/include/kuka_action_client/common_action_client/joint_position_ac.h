#ifndef KUKA_ACTION_CLIENT___JOINT_POSITION_AC_H_
#define KUKA_ACTION_CLIENT___JOINT_POSITION_AC_H_

#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ac{

class Joint_position_ac{

    typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

public:

    Joint_position_ac();

    ~Joint_position_ac();

    void wait_for_server();

    void call_action(const std::vector<double> &joint_position_target);

    actionlib::SimpleClientGoalState getState();

private:

    trajectory_msgs::JointTrajectory                joint_traj_msg;
    control_msgs::FollowJointTrajectoryGoal         goal;
    TrajClient* traj_client_;


};

}

#endif
