#include "lwr_ros_interface/default_topics.h"
namespace ros_controller_interface{

/// Cartesian default topics
const std::string topics::EE_KUKA_POSE_TOPIC    = "/KUKA/Pose";
const std::string topics::EE_STATE_POSE_TOPIC   = "/joint_to_cart/est_ee_pose";
const std::string topics::EE_STATE_FT_TOPIC     = "/joint_to_cart/est_ee_ft";

/// Go to state-transformer
const std::string topics::EE_CMD_POSE_TOPIC     = "/cart_to_joint/des_ee_pose";
const std::string topics::EE_CMD_FT_TOPIC       = "/cart_to_joint/des_ee_ft";
const std::string topics::EE_CMD_VEL_TOPIC      = "/cart_to_joint/des_ee_vel";
const std::string topics::EE_CMD_STIFF_TOPIC    = "/cart_to_joint/des_ee_stiff";

const std::string topics::BASE_LINK             = "/base_link";

/// Joint default topics (directly via bridge to the robot, have to take care of smoothing)


const std::string topics::J_SENSE_TOPIC                 = "/KUKA/joint_states";
const std::string topics::J_CMD_TOPIC                   = "/KUKA/joint_states";

}
