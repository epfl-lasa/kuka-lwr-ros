#ifndef KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_
#define KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_

#include "kuka_action_server/default_services.h"
#include "kuka_action_server/default_types.h"
#include <controller_manager/controller_manager.h>


namespace asrv{


/**
 * @brief The Base_action_server class : Interface to be implemented by user. Each
 * control policy, search method, robot motion heuristic, etc.. has to inherit
 * The Base_action_server class.
 */
class Base_action_server{

public:

    Base_action_server(ros::NodeHandle& nh){
        cmanager_switch_service = nh.serviceClient<controller_manager_msgs::SwitchController>("/lwr/controller_manager/switch_controller");
        cmanager_list_service   = nh.serviceClient<controller_manager_msgs::ListControllers>("/lwr/controller_manager/list_controllers");
    }

    /**
     * @brief execute_CB : server action callbac, this function holds the implementation of the
     *                     robots control policy. In this function the user has to send control
     *                     information to the robot.
     * @param as_        : reference to the action server
     */
    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal) = 0;


    bool activate_controller(const std::string& target_controller){


        cmanager_list_service.call(list_msg.request,list_msg.response);
        ros::spinOnce();
        std::vector<controller_manager_msgs::ControllerState>& controllers = list_msg.response.controller;

        int index = has_controller(controllers,target_controller);
        if(index >= 0 && index < static_cast<int>(controllers.size()) ){

            if(controllers[index].state != "running"){
                std::vector<std::string> running_controllers;
                get_running(controllers,running_controllers);
                switch_msg.request.start_controllers = {{target_controller}};
                switch_msg.request.stop_controllers  = running_controllers;
                switch_msg.request.strictness        = controller_manager_msgs::SwitchController::Request::STRICT;
                cmanager_switch_service.call(switch_msg.request,switch_msg.response);

                if( switch_msg.response.ok == true){
                    ROS_INFO("controller switch was successfull");
                    return true;
                }else{
                    ROS_INFO("controller switch failed!");
                    return false;
                }
            }else{
                ROS_INFO("controller [%s] is allready running",target_controller.c_str());
                return true;
            }

        }else{
            ROS_ERROR("No such controller [%s] available",target_controller.c_str());
            return false;
        }



    }


    int has_controller(const std::vector<controller_manager_msgs::ControllerState>& controllers,const std::string& name){
        for(std::size_t i = 0; i < controllers.size();i++){
            if(controllers[i].name == name){
                return i;
            }
        }
        return -1;
    }

    void get_running(const std::vector<controller_manager_msgs::ControllerState>& controllers,std::vector<std::string>& running_list){
        running_list.clear();
        running_list.resize(0);
        for(std::size_t i = 0; i < controllers.size();i++){
            if((controllers[i].state == "running") && (controllers[i].name != "kuka_joint_state_controller")){
                running_list.push_back(controllers[i].name);
            }
        }
    }


protected:

    ros::ServiceClient                          cmanager_switch_service;
    ros::ServiceClient                          cmanager_list_service;

    controller_manager_msgs::SwitchController   switch_msg;
    controller_manager_msgs::ListControllers    list_msg;



};

}


#endif
