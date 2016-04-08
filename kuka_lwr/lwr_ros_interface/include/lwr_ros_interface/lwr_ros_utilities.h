#ifndef KUKA_ACTION_SERVER_DEFAULT_TOPICS_H_
#define KUKA_ACTION_SERVER_DEFAULT_TOPICS_H_

#include <string>
#include <ros/ros.h>
#include <Eigen/Eigen>

namespace ros_controller_interface{


namespace utilities{

void inline ros_print_joint(double time,const Eigen::VectorXd& data){
    if(data.size() == 7){
        ROS_INFO_STREAM_THROTTLE(time,"joint: " << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6]);
    }else{
        ROS_WARN_STREAM_THROTTLE(time,"data.size != 7: " << data.size() << " [ros_print_joint]!");
    }
}


}

}

#endif
