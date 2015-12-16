#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include <functional>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace asrv{

Kuka_goto_cart_as::Kuka_goto_cart_as(ros::NodeHandle& nh)
    : Base_ee_action(nh),
      Base_action_server(nh)
{

    reachingThreshold       = 0.001;     // [m]
    orientationThreshold    = 0.01;  // [rad]
    dt                      = 1.0/100.0;

  /*  rviz_arrow.resize(1);
    rviz_arrow[0].shaft_diameter = 0.005;
    rviz_arrow[0].head_diameter  = 0.01;
    rviz_arrow[0].head_length    = 0.015;

    std::vector<tf::Vector3> colors(1);
    colors[0] = tf::Vector3(1,0,0);

    rviz_direction.initialise(world_frame,rviz_arrow);
    rviz_direction.set_color(colors);
    rviz_direction.scale = 0.05;

    rviz_points.resize(1);

    rviz_points_viz.scale = 0.01;
    rviz_points_viz.r     = 0;
    rviz_points_viz.g     = 1;
    rviz_points_viz.b     = 0;
    rviz_points_viz.initialise(world_frame,rviz_points);*/

}

bool Kuka_goto_cart_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){


        activate_controller("one_task_inverse_kinematics");


        tf::Transform trans_att;

        trans_att.setRotation(tf::Quaternion(goal->target_frame.rotation.x,goal->target_frame.rotation.y,
                                             goal->target_frame.rotation.z,goal->target_frame.rotation.w));
        trans_att.setOrigin(tf::Vector3(goal->target_frame.translation.x, goal->target_frame.translation.y,
                                        goal->target_frame.translation.z));


        ee_pos_msg.position.x       = goal->target_frame.translation.x;
        ee_pos_msg.position.y       = goal->target_frame.translation.y;
        ee_pos_msg.position.z       = goal->target_frame.translation.z;
        ee_pos_msg.orientation.x    = trans_att.getRotation().x();
        ee_pos_msg.orientation.y    = trans_att.getRotation().y();
        ee_pos_msg.orientation.z    = trans_att.getRotation().z();
        ee_pos_msg.orientation.w    = trans_att.getRotation().w();

        ROS_INFO("des origin (%f %f %f)",ee_pos_msg.position.x,ee_pos_msg.position.y,ee_pos_msg.position.z);
       // ROS_INFO("des orient (%f %f %f)",des_ee_pos_.orientation.roll,des_ee_pos_.orientation.pitch,des_ee_pos_.orientation.yaw);

        sendPose(ee_pos_msg);

        ros::Rate loop_rate(100);
        bool success = true;
        while(ros::ok()) {


            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Preempted");
                as_.setPreempted();
                success = false;
                break;
            }
            loop_rate.sleep();
        }

        return success;
}


Eigen::Vector3f Kuka_goto_cart_as::d2qw(Eigen::Vector4f  q,  Eigen::Vector4f  dq){
    // -- DQ2W Converts Quaterion rates to angular velocity --//
    Eigen::Vector3f w;
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    Eigen::MatrixXf W(3,4);
    W << -qx,  qw, -qz,  qy,
         -qy,  qz,  qw, -qx,
         -qz, -qy,  qx,  qw;

    w = 2 * W * dq;

    std::cout << w << std::endl;
    return w;
}





}
