/* ============================================================
 *
 * This file is a part of SpaceNav (CoSiMA) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#ifndef _SPACENAV_OROCOS_HPP_
#define _SPACENAV_OROCOS_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <string>
#include "../spacenav-hid.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

// RST-RT
#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/geometry/Rotation.hpp>

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

  void resetOrientation(float w, float x, float y, float z);

  void resetPoseToInitial();

  void resetPose(rstrt::geometry::Pose pose);

  void setInitialRotation(rstrt::geometry::Rotation ir);

protected:
  cosima::hw::SpaceNavHID *interface;

  virtual int getFileDescriptor();

  RTT::OutputPort<Eigen::VectorXf> out_6d_port;
  Eigen::VectorXf out_6d_var;

  RTT::OutputPort<rstrt::geometry::Pose> out_pose_port;
  rstrt::geometry::Pose out_pose_var;

  RTT::InputPort<rstrt::geometry::Pose> in_current_pose_port;
  rstrt::geometry::Pose in_current_pose_var, initial_pose_var;
  RTT::FlowStatus in_current_pose_flow;

private:
  cosima::hw::SpaceNavValues values;
  cosima::hw::SpaceNavValues rawValues;

  rstrt::geometry::Rotation initial_rotation;

  float offsetTranslation;
  float offsetOrientation;

  bool button1_old, button2_old;

  bool enableX, enableY, enableZ, enableA, enableB, enableC;

  int sensitivity;

  float cageMinX, cageMinY, cageMinZ, cageMaxX, cageMaxY, cageMaxZ;
  bool isCageActive;
};

} // namespace hw
} // namespace cosima

#endif
