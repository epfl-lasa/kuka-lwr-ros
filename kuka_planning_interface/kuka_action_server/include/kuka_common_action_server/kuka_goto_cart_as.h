#ifndef KUKA_ACTION_SERVER_KUKA_GOTO_CART_AS_H_
#define KUKA_ACTION_SERVER_KUKA_GOTO_CART_AS_H_

/**
  *  Kuka goto cartesian action server
  *
  *  Robot moves from its current end-effector frame of reference (6D, position + orientation)
  *  to a target frame of reference. The positions along the trajectory are computed through
  *  inverse kinematics.
  *
  **/

#include "kuka_action_server/base_action_server.h"
#include "kuka_action_server/base_ee_action.h"
#include "kuka_action_server/base_j_action.h"
#include "kuka_common_action_server/action_initialiser.h"
#include "kuka_common_action_server/speed_generator.h"
//#include "visualise/vis_vector.h"
//#include "visualise/vis_points.h"
#include "lwr_controllers/PoseRPY.h"


namespace asrv{

class Kuka_goto_cart_as : public Base_ee_action , public Base_action_server{

public:

    Kuka_goto_cart_as(ros::NodeHandle&  nh);

    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal);

private:

    Eigen::Vector3f d2qw(Eigen::Vector4f q,Eigen::Vector4f dq);

public:

    Speed_generator         speed_generator;

private:

    std::string             action_name;
    std::string             world_frame;

    geometry_msgs::Pose     des_ee_pose;    /// desired end-effector position
    geometry_msgs::Twist    des_ee_vel;     /// desired end-effector velocities

    double                  dt;

    double                  reachingThreshold;
    double                  orientationThreshold;

  //  lwr_controllers::PoseRPY des_ee_pos_;

   /* opti_rviz::Vis_vectors          rviz_direction;
    std::vector<opti_rviz::Arrow>   rviz_arrow;

    std::vector<tf::Vector3>        rviz_points;
    opti_rviz::Vis_points           rviz_points_viz;*/


};

}

#endif
