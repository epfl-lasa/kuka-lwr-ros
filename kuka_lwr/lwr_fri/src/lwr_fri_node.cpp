#include "ros/package.h"
#include <iostream>
#include <thread>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Bool.h>

#include "lwr_fri/lwr_fri_interface.h"
#include "lwr_fri/LWRRobot_FRI.h"
#include "lwr_fri/FRI.h"

bool g_quit = false;

void quitRequested(int sig)
{
    g_quit = true;
}

bool isStopPressed = false;
bool wasStopHandled = true;
void eStopCB(const std_msgs::BoolConstPtr& e_stop_msg)
{
    isStopPressed = e_stop_msg->data;
}

// Get the URDF XML from the parameter server
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
    std::string urdf_string;
    std::string robot_description = "/robot_description";

    // search and wait for robot_description on param server
    while (urdf_string.empty())
    {
        std::string search_param_name;
        if (model_nh_.searchParam(param_name, search_param_name))
        {
            ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
                                " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

            model_nh_.getParam(search_param_name, urdf_string);
        }
        else
        {
            ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
                                " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

            model_nh_.getParam(param_name, urdf_string);
        }

        usleep(100000);
    }
    ROS_DEBUG_STREAM_NAMED("LWRHWFRI", "Received URDF from param server, parsing...");

    return urdf_string;
}

void populateFloatArrayMatrix(std_msgs::Float64MultiArray &msg, float **matrix, size_t d1, size_t d2) {
  int ii = 0;
  if (msg.data.size() != (d1*d2)) msg.data.resize(d1*d2);
  for (int j = 0; j < d1; ++j) {
    for (int k = 0; k < d2; ++k) {
      msg.data[ii++] = matrix[j][k];
    }
  }
}

int main(int argc, char** argv){


    std::cout<< "==== KUKA FRI BRIDGE ===" << std::endl;

    // initialize ROS
    ros::init(argc, argv, "KUKA_FRI_BRIDGE", ros::init_options::NoSigintHandler);

    // ros spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // custom signal handlers
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    // create a node
    ros::NodeHandle lwr_nh;

    /// Start FRI interface
    kfb::Fri_interface fri_interface(lwr_nh);
    fri_interface.start_fri();

    /// Load robot description
    std::string urdf_string = getURDF(lwr_nh, "/lwr/robot_description");
    kfb::LWRRobot_FRI lwr_robot_fri(fri_interface.mFRI);
    lwr_robot_fri.create("lwr", urdf_string);
    lwr_robot_fri.init();

    //the controller manager
    controller_manager::ControllerManager manager(&lwr_robot_fri, lwr_nh);

    struct timespec ts = {0, 0};
    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period(1.0);
    ros::Rate rate(2000.0);

    // Publish inertia matrix and Jacobian
    float **inertia_matrix =  new float*[7];
    for(int i = 0; i < 7; ++i) {
        inertia_matrix[i] = new float[7];
    }
    float **jacobian_matrix =  new float*[6];
    for(int i = 0; i < 7; ++i) {
        jacobian_matrix[i] = new float[7];
    }

    std_msgs::Float64MultiArray inertia_msg;
    std_msgs::Float64MultiArray jacobian_msg;

    ros::Publisher inertia_pub  = lwr_nh.advertise<std_msgs::Float64MultiArray>("/lwr/inertia", 1);
    ros::Publisher jacobian_pub  = lwr_nh.advertise<std_msgs::Float64MultiArray>("/lwr/jacobian", 1);

    ROS_INFO("==== READY TO START ====");
    double elapsed_time_status = 0;
    double elapsed_time_jacobian_inertia = 0;
    while( !g_quit )
    {
        // get the time / period
        if (!clock_gettime(CLOCK_REALTIME, &ts))
        {
            now.sec = ts.tv_sec;
            now.nsec = ts.tv_nsec;
            period = now - last;
            last = now;
        }
        else
        {
            ROS_FATAL("Failed to poll realtime clock!");
            break;
        }

        lwr_robot_fri.read(now,period);

        manager.update(now, period);

        lwr_robot_fri.write(now, period);

        // Additional publishers apart from ROS control
        elapsed_time_status = elapsed_time_status + period.toSec();
        elapsed_time_jacobian_inertia = elapsed_time_jacobian_inertia + period.toSec();

        fri_interface.mFRI->GetCurrentMassMatrix(inertia_matrix);
        fri_interface.mFRI->GetCurrentJacobianMatrix(jacobian_matrix);
        populateFloatArrayMatrix(inertia_msg, inertia_matrix, 7, 7);
        populateFloatArrayMatrix(jacobian_msg, jacobian_matrix, 6, 7);

        // Publish FRI status with 50 Hz
        if (elapsed_time_status > 0.02) {
            fri_interface.publish(lwr_robot_fri);
            elapsed_time_status=0.0;
        }
        // Publish jacobian and inertia with 1 kHz
        if (elapsed_time_jacobian_inertia > 0.001) {
            inertia_pub.publish(inertia_msg);
            jacobian_pub.publish(jacobian_msg);
            elapsed_time_jacobian_inertia=0.0;
        }



        rate.sleep();
    }

    for(int i = 0; i < 7; ++i) {
        delete [] inertia_matrix[i];
    }
    for(int i = 0; i < 6; ++i) {
        delete [] jacobian_matrix[i];
    }

    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();
    std::cerr<<"This node was killed!"<<std::endl;


    return 0;
}
