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
#include "utilities.h"


#include <memory>
#include <deque>

#define FRI_CONN_TIMEOUT_SEC	30

namespace kfb
{

class LWRRobot_FRI : public lwr_hw::LWRHW
{

public:

    struct LWRMeasurement{
      std::vector<double> JointPositions;
      std::vector<double> JointVelocities;
      std::vector<double> JointAccelerations;
      std::vector<double> JointTorques;
      double t;
    };

public:

  LWRRobot_FRI(boost::shared_ptr<FastResearchInterface>& mFRI);

  ~LWRRobot_FRI();

  bool init();

  bool SetControlMode(ControlStrategy desiredMode);

  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);

private:

  void safe_joint_target_update(ros::Time time, ros::Duration period);

private:

  boost::shared_ptr<FastResearchInterface> mFRI;

  FRI_STATE                     mCurrentFRI_STATE;
  FRI_CTRL                      mCurrentFRI_Control;
  FRI_CTRL                      mDesiredFRI_Control;
  ControlStrategy               mDesired_control_strategy;
  volatile bool                 mSwitched;

  std::deque<LWRMeasurement>    MeasurementHistory;

  double                        **massMatrix;

  int                           ResultValue;

};

}


#endif
