#ifndef ROS_PARAM_PARSER_H_
#define ROS_PARM_PARSER_H_

// ROS

#include <ros/ros.h>

// STL

#include <string>
#include <map>
#include <vector>

namespace pps{

class Parser{

public:

static bool parser_string(ros::NodeHandle& nh,std::map<std::string,std::string>& param_name_value);

static void parser_print(std::map<std::string,std::string>& param_name_value);

};

}



#endif
