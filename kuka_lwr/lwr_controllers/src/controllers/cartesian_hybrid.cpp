#include "controllers/cartesian_hybrid.h"

namespace controllers{


Cartesian_hybrid::Cartesian_hybrid(ros::NodeHandle &nh,controllers::Change_ctrl_mode& change_ctrl_mode):
    Base_controllers(lwr_controllers::CTRL_MODE::CART_POSITION),change_ctrl_mode(change_ctrl_mode)
{

    sub_command_pose_      = nh.subscribe("cart_hybrid/cmd_pos",      1, &Cartesian_hybrid::command_cart_pos,     this);


}

void Cartesian_hybrid::stop(){

}

void Cartesian_hybrid::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{

   // ROS_INFO("================================================> command_cart_pos callback!");
    KDL::Frame frame_des_(
                KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
                KDL::Vector(msg->position.x,msg->position.y,msg->position.z));

    x_des_      = frame_des_;
    cmd_flag_   = true;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_POSITION);
    }
    bFirst            = true;

}


}
