#ifndef ROS_PARAM_PARSER_H_
#define ROS_PARM_PARSER_H_

// ROS

#include <ros/ros.h>

// STL

#include <string>
#include <map>
#include <vector>

namespace pps{

bool parser_string(ros::NodeHandle& nh,std::map<std::string,std::string>& param_name_value){
      bool ret = true;
      std::map<std::string,std::string>::iterator it;
      for (it=param_name_value.begin(); it!=param_name_value.end(); ++it){
          std::string topic = (it->first);
          std::string out;
          if(! nh.getParam(topic,out) ){
              std::string message = "Must provide on the parameter server: " + (it->first);
              ROS_ERROR("%s",message.c_str());
          }
          it->second = out;
      }
      return ret;
}

void parser_print(std::map<std::string,std::string>& param_name_value){
    std::map<std::string,std::string>::iterator it;
    std::cout<< "--- parameters ---" << std::endl;
    for (it=param_name_value.begin(); it!=param_name_value.end(); ++it){
        std::cout<< it->first << " : " << it->second << std::endl;
    }



}

}



#endif
