#include "kuka_action_server/base_ee_action.h"

namespace asrv{

Base_ee_action::Base_ee_action(ros::NodeHandle&   nh){

   /* ee_ft.resize(6);
    sub_        = nh.subscribe<geometry_msgs::PoseStamped>(ee_state_pos_topic, 1, &Base_ee_action::eeStateCallback,this);
    pub_        = nh.advertise<geometry_msgs::PoseStamped>(ee_cmd_pos_topic, 1);
    pub_ft_     = nh.advertise<geometry_msgs::WrenchStamped>(ee_cmd_ft_topic, 1);
    pub_vel_    = nh.advertise<geometry_msgs::TwistStamped>(ee_cmd_vel_topic, 1);
    */
    position_pub = nh.advertise<geometry_msgs::Pose>("/lwr/one_task_inverse_kinematics/command_pos", 1);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/lwr/one_task_inverse_kinematics/command_vel", 1);
    pose_sub     = nh.subscribe("/lwr/ee_pose",1,&Base_ee_action::pose_callback,this);
    b_received   = false;

}

void Base_ee_action::sendPose(const geometry_msgs::Pose & pose_msg) {
   /* ee_pos_msg.position.x = pose_msg.getOrigin().x();
    ee_pos_msg.position.y = pose_msg.getOrigin().y();
    ee_pos_msg.position.z = pose_msg.getOrigin().z();

    tmp.setRotation(pose_msg.getRotation());
    tmp.getRPY(ee_pos_msg.orientation.roll,ee_pos_msg.orientation.pitch,ee_pos_msg.orientation.yaw);*/
    position_pub.publish(pose_msg);
}

void Base_ee_action::sendVel(const geometry_msgs::Twist& twist_){
    velocity_pub.publish(twist_);
}




void Base_ee_action::pose_callback(const geometry_msgs::PoseConstPtr &msg){

    ee_pose_current.setOrigin(tf::Vector3(msg->position.x,msg->position.y,msg->position.z));
    ee_pose_current.setRotation(tf::Quaternion(msg->orientation.x,
                                               msg->orientation.y,
                                               msg->orientation.z,
                                               msg->orientation.w));
    b_received = true;
}

}
