#include "lwr_ros_client/ros_param_parser.h"

namespace pps{


bool Parser::parser_string(ros::NodeHandle& nh,std::map<std::string,std::string>& param_name_value){
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

void Parser::parser_print(std::map<std::string,std::string>& param_name_value){
    std::map<std::string,std::string>::iterator it;
    std::cout<< "--- parameters ---" << std::endl;
    for (it=param_name_value.begin(); it!=param_name_value.end(); ++it){
        std::cout<< it->first << " : " << it->second << std::endl;
    }

}


}
