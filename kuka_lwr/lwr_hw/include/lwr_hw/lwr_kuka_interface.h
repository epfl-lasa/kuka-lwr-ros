#ifndef HARDWARE_INTERFACE_LWR_KUKA_INTERFACE_H
#define HARDWARE_INTERFACE_LWR_KUKA_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

/** A KUKA joint handle used to read the state of a single joint (position, torque, stifness & dampin). */
class KukaStateHandle
{
public:
  KukaStateHandle() : name_(), pos_(0), vel_(0), acc_(0), eff_(0), sti_(0), dam_(0) {}

  /**
   * \param name The name of the joint
   * \param pos A pointer to the storage for this joint's position
   * \param vel A pointer to the storage for this joint's velocity
   * \param eff A pointer to the storage for this joint's effort (force or torque)
   */
  KukaStateHandle(const std::string& name, const double* pos, const double* vel, const double* acc, const double* eff, const double* dam, const double* sti)
    : name_(name), pos_(pos), vel_(vel), acc_(acc), eff_(eff), dam_(dam), sti_(sti)
  {
    if (!pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
    }
    if (!vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
    }
    if (!acc)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Acceleration data pointer is null.");
    }
    if (!eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Effort data pointer is null.");
    }
    if (!dam)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Damping data pointer is null.");
    }
    if (!sti)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Stiffness data pointer is null.");
    }
  }

  std::string getName()     const {return name_;}
  double getPosition()      const {assert(pos_); return *pos_;}
  double getVelocity()      const {assert(vel_); return *vel_;}
  double getAcceleration()  const {assert(acc_); return *acc_;}
  double getTorque()        const {assert(eff_); return *eff_;}
  double getStiffness()     const {assert(sti_); return *sti_;}
  double getDamping()       const {assert(dam_); return *dam_;}

private:
  std::string name_;
  const double* pos_;   // position
  const double* vel_;   // velocity
  const double* acc_;   // acceleration
  const double* eff_;   // torque
  const double* sti_;   // stiffness
  const double* dam_;   // damping



};


/** \brief A handle used to read and command a single joint. */
class KUKAJointHandle : public KukaStateHandle
{
public:
  KUKAJointHandle() : KukaStateHandle(), pos_cmd_(0), eff_cmd_(0), sti_cmd_(0), dam_cmd_(0){}

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  KUKAJointHandle(const KukaStateHandle& js, double* pos_cmd, double* eff_cmd, double* sti_cmd, double* dam_cmd)
    : KukaStateHandle(js), pos_cmd_(pos_cmd), eff_cmd_(eff_cmd), sti_cmd_(sti_cmd), dam_cmd_(dam_cmd)
  {
    if (!pos_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [position] data pointer is null.");
    }

    if (!eff_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [torque] data pointer is null.");
    }

    if (!sti_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [stiffness] data pointer is null.");
    }

    if (!dam_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [damping] data pointer is null.");
    }
  }

  void setCommandPosition(double command)   {assert(pos_cmd_); *pos_cmd_ = command;}
  void setCommandTorque(double command)     {assert(eff_cmd_); *eff_cmd_ = command;}
  void setCommandStiffness(double command)  {assert(sti_cmd_); *sti_cmd_ = command;}
  void setCommandDamping(double command)    {assert(dam_cmd_); *dam_cmd_ = command;}


  double getCommandPosition()   const {assert(pos_cmd_); return *pos_cmd_;}
  double getCommandTorque()     const {assert(eff_cmd_); return *eff_cmd_;}
  double getCommandStiffness()  const {assert(sti_cmd_); return *sti_cmd_;}
  double getCommandDamping()    const {assert(dam_cmd_); return *dam_cmd_;}

private:

  double* pos_cmd_;
  double* eff_cmd_;
  double* sti_cmd_;
  double* dam_cmd_;

};


class JointKUKAStateInterface : public HardwareResourceManager<KukaStateHandle> {};

class JointKUKACommandInterface : public HardwareResourceManager<KUKAJointHandle, ClaimResources> {};

class KUKAJointInterface : public JointKUKACommandInterface {};

}

#endif
