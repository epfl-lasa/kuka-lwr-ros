#ifndef KUKA_FRI_BRIDGE_LWROBOTFRI_H_
#define KUKA_FRI_BRIDGE_LWROBOTFRI_H_

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRI remote hooks
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include <thread>

#include "FastResearchInterface.h"
#include "friudp.h"
#include "friremote.h"
#include "fricomm.h"

#include "utilities.h"


#include <memory>

#define FRI_CONN_TIMEOUT_SEC	30

namespace kfb
{

class LWRRobot_FRI : public lwr_hw::LWRHW
{

public:

  LWRRobot_FRI(boost::shared_ptr<FastResearchInterface>& mFRI);

  ~LWRRobot_FRI();

 // void setPort(int port);

 // void setIP(std::string hintToRemoteHost);

  // Init, read, and write, with FRI hooks
  bool init();

  bool SetControlMode(ControlStrategy desiredMode);

  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);

  void actual_switch();

private:

  void safe_joint_target_update(ros::Time time, ros::Duration period);

private:

  boost::shared_ptr<FastResearchInterface> mFRI;
  boost::shared_ptr<friRemote> device_;


  FRI_STATE                     mCurrentFRI_STATE;
  FRI_CTRL                      mCurrentFRI_Control;
  FRI_CTRL                      mDesiredFRI_Control;
  ControlStrategy               mDesired_control_strategy;

  // Parameters
  int port_;
  bool port_set;
  std::string hintToRemoteHost_;
  bool ip_set;

  float* measured_joint_position;


  void startFRI();
  void stopFRI();
};

}


#endif
