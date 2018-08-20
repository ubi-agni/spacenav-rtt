#ifndef _SPACENAV_OROCOS_HPP_
#define _SPACENAV_OROCOS_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <string>
#include "../spacenav-hid.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

// // RST-RT
#include <rst-rt/geometry/Pose.hpp>

namespace cosima
{
namespace hw
{

class SpaceNavOrocos : public RTT::TaskContext
{

public:
  SpaceNavOrocos(std::string const &name = "SpaceNavOrocos");

  // ~SpaceNavOrocos();

  bool configureHook();

  bool startHook();

  void updateHook();

  // void errorHook();

  void stopHook();

  void cleanupHook();

  void displayStatus();

protected:
  cosima::hw::SpaceNavHID *interface;

  virtual int getFileDescriptor();

  RTT::OutputPort<Eigen::VectorXf> out_6d_port;
  Eigen::VectorXf out_6d_var;

  RTT::OutputPort<rstrt::geometry::Pose> out_pose_port;
  rstrt::geometry::Pose out_pose_var;

  RTT::InputPort<rstrt::geometry::Pose> in_current_pose_port;
  rstrt::geometry::Pose in_current_pose_var;
  RTT::FlowStatus in_current_pose_flow;

private:
  cosima::hw::SpaceNavValues values;
  cosima::hw::SpaceNavValues rawValues;

  float offsetTranslation;
  float offsetOrientation;

  bool button1_old, button2_old;
};

} // namespace hw
} // namespace cosima

#endif
