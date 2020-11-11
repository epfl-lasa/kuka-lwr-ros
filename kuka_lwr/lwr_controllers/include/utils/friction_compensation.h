//
// Created by dominic on 11.11.20.
//

#ifndef SRC_FRICTION_COMPENSATION_H
#define SRC_FRICTION_COMPENSATION_H

#include "ros/package.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace lwr_controllers {
class FrictionCompensation {

public:
  static void reloadParameters();
  static bool compensateFriction(double thrott_time, KDL::JntArray &tau);

private:
  static void loadParameters(std::size_t n_joints);
};

static bool reload_parameters_ = false;
static bool file_ok_ = false;

static std::vector<double> coulomb_friction_, viscous_friction_;

void FrictionCompensation::loadParameters(std::size_t n_joints) {
  std::string paramFilePath = ros::package::getPath("lwr_controllers");
  paramFilePath += "/param/friction_comp.yaml";
  YAML::Node params;
  try {
    params = YAML::LoadFile(paramFilePath);
    file_ok_ = true;
  } catch (const YAML::Exception &ex) {
    ROS_WARN_STREAM("[FrictionCompensation::loadParameters] " << ex.what());
    ROS_WARN_STREAM(
        "[FrictionCompensation::loadParameters] Impossible to do joint "
        "friction "
        "compensation. Check the parameter file named 'friction_comp.yaml' "
        "in the 'param' directory of the 'lwr_controllers' package.");
    file_ok_ = false;
  }
  if (file_ok_) {
    coulomb_friction_.resize(n_joints);
    viscous_friction_.resize(n_joints);
    for (std::size_t i = 0; i < n_joints; i++) {
      try {
        coulomb_friction_[i] =
            params["joint_" + std::to_string(i + 1)]["offset"].as<double>();
        ROS_WARN_STREAM(coulomb_friction_[i]);
        viscous_friction_[i] =
            params["joint_" + std::to_string(i + 1)]["slope"].as<double>();
      } catch (YAML::Exception &ex) {
        ROS_WARN_STREAM("[FrictionCompensation::loadParameters] " << ex.what());
        ROS_WARN_STREAM(
            "[FrictionCompensation::loadParameters] Impossible to do joint "
            "friction "
            "compensation. Check the parameter file named 'friction_comp.yaml' "
            "in the 'param' directory of the 'lwr_controllers' package.");
        file_ok_ = false;
        return;
      }
    }
    // TODO assert that values are okay???
  }
}

bool FrictionCompensation::compensateFriction(double thrott_time,
                                              KDL::JntArray &tau) {
  if (reload_parameters_) {
    loadParameters(tau.rows());
    reload_parameters_ = false;
  }
  if (file_ok_) {
    ROS_INFO_STREAM_THROTTLE(thrott_time, "i would apply compensation here");
    //    for(size_t i = 0; i < 7; i++){
    //
    //    }
    return true;
  } else {
    return false;
  }
}

void FrictionCompensation::reloadParameters() { reload_parameters_ = true; }

} // namespace lwr_controllers

#endif // SRC_FRICTION_COMPENSATION_H
