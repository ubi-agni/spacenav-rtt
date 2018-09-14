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

#ifdef USE_RSTRT
// RST-RT
#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/geometry/Rotation.hpp>
#endif


#ifdef USE_ROS
// ROS messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#endif

namespace cosima
{
namespace hw
{
  
  
#ifdef USE_RSTRT
typedef rstrt::geometry::Pose SpaceNavMsgRot;
typedef rstrt::geometry::Rotation SpaceNavMsgPose;
#else
 #ifdef USE_ROS
  typedef geometry_msgs::Pose SpaceNavMsgPose;
  typedef geometry_msgs::Quaternion SpaceNavMsgRot;
 #endif
#endif


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

#if defined USE_RSTRT || defined USE_ROS
  void resetOrientation(float w, float x, float y, float z);

  void resetPoseToInitial();

  void resetPose(SpaceNavMsgPose pose);

  void setInitialRotation(SpaceNavMsgRot ir);
#endif


protected:
  cosima::hw::SpaceNavHID *interface;

  virtual int getFileDescriptor();

  RTT::OutputPort<Eigen::VectorXf> out_6d_port;
  Eigen::VectorXf out_6d_var;

#if defined USE_RSTRT || defined USE_ROS
  RTT::OutputPort<SpaceNavMsgPose> out_pose_port;
  SpaceNavMsgPose out_pose_var;

  RTT::InputPort<SpaceNavMsgPose> in_current_pose_port;
  SpaceNavMsgPose in_current_pose_var, initial_pose_var;
  RTT::FlowStatus in_current_pose_flow;
#endif


private:
  void process_rstrst();
  void process_ros();

  cosima::hw::SpaceNavValues values;
  cosima::hw::SpaceNavValues rawValues;

#if defined USE_RSTRT || defined USE_ROS
  SpaceNavMsgRot initial_rotation;
#endif


  std::string devicePath;
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
