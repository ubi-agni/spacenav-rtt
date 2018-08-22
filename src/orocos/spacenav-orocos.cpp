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

#include "spacenav-orocos.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp>

using namespace cosima::hw;

SpaceNavOrocos::SpaceNavOrocos(std::string const &name) : RTT::TaskContext(name),
                                                          offsetTranslation(0.001),
                                                          offsetOrientation(0.001),
                                                          button1_old(false),
                                                          button2_old(false),
                                                          enableX(true),
                                                          enableY(true),
                                                          enableZ(true),
                                                          enableA(true),
                                                          enableB(true),
                                                          enableC(true),
                                                          sensitivity(160),
                                                          cageMinX(0),
                                                          cageMinY(0),
                                                          cageMinZ(0),
                                                          cageMaxX(0.5),
                                                          cageMaxY(0.5),
                                                          cageMaxZ(0.6),
                                                          isCageActive(false)
{
    addOperation("displayStatus", &SpaceNavOrocos::displayStatus, this).doc("Display the current status of this component.");
#ifdef USE_RSTRT
    addOperation("resetOrientation", &SpaceNavOrocos::resetOrientation, this).doc("Reset the orientation to new quaternion values.");
    addOperation("resetPoseToInitial", &SpaceNavOrocos::resetPoseToInitial, this).doc("Reset the entire pose to the initial one.");
    addOperation("resetPose", &SpaceNavOrocos::resetPose, this).doc("Reset the pose to a new one.");
    addOperation("setInitialRotation", &SpaceNavOrocos::setInitialRotation, this).doc("Set the rotation before the component is started.");
#endif

    addProperty("sensitivity", sensitivity);

    addProperty("offsetTranslation", offsetTranslation);
    addProperty("offsetOrientation", offsetOrientation);

    addProperty("enableX", enableX);
    addProperty("enableY", enableY);
    addProperty("enableZ", enableZ);
    addProperty("enableA", enableA);
    addProperty("enableB", enableB);
    addProperty("enableC", enableC);

    addProperty("cageMinX", cageMinX);
    addProperty("cageMinY", cageMinY);
    addProperty("cageMinZ", cageMinZ);
    addProperty("cageMaxX", cageMaxX);
    addProperty("cageMaxY", cageMaxY);
    addProperty("cageMaxZ", cageMaxZ);
    addProperty("isCageActive", isCageActive);
    interface = new SpaceNavHID();
}

// SpaceNavOrocos::~SpaceNavOrocos() {
//     if (interface) {
//         delete interface;
//     }
// }

bool SpaceNavOrocos::configureHook()
{
    if (!interface->init())
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Unable to access Space Nav at " << getFileDescriptor() << RTT::endlog();
        return false;
    }

    if (this->getPort("out_6d_port"))
    {
        this->ports()->removePort("out_6d_port");
    }
    out_6d_var = Eigen::VectorXf(interface->getNumAxes());
    out_6d_var.setZero();
    out_6d_port.setName("out_6d_port");
    out_6d_port.doc("Output port for 6D command vector");
    out_6d_port.setDataSample(out_6d_var);
    ports()->addPort(out_6d_port);

#ifdef USE_RSTRT
    if (this->getPort("out_pose_port"))
    {
        this->ports()->removePort("out_pose_port");
    }
    out_pose_var = rstrt::geometry::Pose();
    out_pose_port.setName("out_pose_port");
    out_pose_port.doc("Output port for pose command vector");
    out_pose_port.setDataSample(out_pose_var);
    ports()->addPort(out_pose_port);

    if (this->getPort("in_current_pose_port"))
    {
        this->ports()->removePort("in_current_pose_port");
    }
    in_current_pose_var = rstrt::geometry::Pose();
    initial_pose_var = rstrt::geometry::Pose();
    in_current_pose_port.setName("in_current_pose_port");
    in_current_pose_port.doc("Input port for the current pose to which the commands should be added.");
    ports()->addPort(in_current_pose_port);
    in_current_pose_flow = RTT::NoData;

    initial_rotation = rstrt::geometry::Rotation();
#endif

    values.reset();
    rawValues.reset();

    // indicate proper setup by flashing the led!
    for (uint i = 0; i < 3; i++)
    {
        interface->setLedState(1);
        usleep(100000);
        interface->setLedState(0);
        usleep(100000);
    }

    return true;
}

int SpaceNavOrocos::getFileDescriptor()
{
    return interface->getFileDescriptor();
}

#ifdef USE_RSTRT
void SpaceNavOrocos::setInitialRotation(rstrt::geometry::Rotation ir)
{
    initial_rotation = ir;
}
#endif

bool SpaceNavOrocos::startHook()
{
#ifdef USE_RSTRT
    in_current_pose_flow = RTT::NoData;
#endif
    RTT::extras::FileDescriptorActivity *activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity)
    {
        activity->watch(getFileDescriptor());
        // get trigger a least every 25 ms
        // activity->setTimeout(25);
        interface->setLedState(1);
        return true;
    }
    return false;
}

// void SpaceNavOrocos::errorHook() {
//     RTT::log(RTT::Fatal) << "ERROR??? TODO" << RTT::endlog();
// }

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void SpaceNavOrocos::updateHook()
{
    interface->getValue(values, rawValues);

    // adjust sensitivity
    values.tx = fabs(values.tx) > sensitivity ? values.tx : 0.0;
    values.ty = fabs(values.ty) > sensitivity ? values.ty : 0.0;
    values.tz = fabs(values.tz) > sensitivity ? values.tz : 0.0;
    values.rx = fabs(values.rx) > sensitivity ? values.rx : 0.0;
    values.ry = fabs(values.ry) > sensitivity ? values.ry : 0.0;
    values.rz = fabs(values.rz) > sensitivity ? values.rz : 0.0;

    // TODO do some scaling!
    if (values.button1 != button1_old)
    {
        button1_old = values.button1;
        if (!button1_old)
        {
            RTT::log(RTT::Error) << "[" << this->getName() << "] "
                                 << "Enabled translation change." << RTT::endlog();
        }
        else
        {
            RTT::log(RTT::Error) << "[" << this->getName() << "] "
                                 << "Disabled translation change." << RTT::endlog();
        }
    }

    if (values.button2 != button2_old)
    {
        button2_old = values.button2;
        if (!button2_old)
        {
            RTT::log(RTT::Error) << "[" << this->getName() << "] "
                                 << "Enabled orientation change." << RTT::endlog();
        }
        else
        {
            RTT::log(RTT::Error) << "[" << this->getName() << "] "
                                 << "Disabled orientation change." << RTT::endlog();
        }
    }

    if (!values.button1)
    {
        out_6d_var(0) = enableX ? sgn(values.tx) * offsetTranslation : 0.0;
        out_6d_var(1) = enableY ? sgn(values.ty) * offsetTranslation : 0.0;
        out_6d_var(2) = enableZ ? sgn(values.tz) * offsetTranslation : 0.0;
    }
    else
    {
        out_6d_var(0) = 0;
        out_6d_var(1) = 0;
        out_6d_var(2) = 0;
    }

    if (!values.button2)
    {
        out_6d_var(3) = enableA ? sgn(values.rx) * offsetOrientation : 0.0;
        out_6d_var(4) = enableB ? sgn(values.ry) * offsetOrientation : 0.0;
        out_6d_var(5) = enableC ? sgn(values.rz) * offsetOrientation : 0.0;
    }
    else
    {
        out_6d_var(3) = 0;
        out_6d_var(4) = 0;
        out_6d_var(5) = 0;
    }

#ifdef USE_RSTRT
    if (!in_current_pose_port.connected())
    {
        // if we do not have a pose to add stuff to, we just return the stuff...
        out_6d_port.write(out_6d_var);
    }
    else
    {
        // if we do have a pose, we treat our values as new delta!
        if (in_current_pose_flow == RTT::NoData)
        {
            // get the ground truth only once!
            in_current_pose_flow = in_current_pose_port.read(in_current_pose_var);
            if (!(initial_rotation.rotation.w() == 0 && initial_rotation.rotation.x() == 0 && initial_rotation.rotation.y() == 0 && initial_rotation.rotation.z() == 0))
            {
                in_current_pose_var.rotation = initial_rotation;
            }
            initial_pose_var = in_current_pose_var;
            return;
        }

        // Eigen::AngleAxisf rollAngle(out_6d_var(3), Eigen::Vector3f::UnitX());
        // Eigen::AngleAxisf pitchAngle(out_6d_var(4), Eigen::Vector3f::UnitY());
        // Eigen::AngleAxisf yawAngle(out_6d_var(5), Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = out_pose_var.rotation.euler2Quaternion(out_6d_var(3), out_6d_var(4), out_6d_var(5));
        // Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;
        // Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

        // Eigen::Quaternionf qBase = Eigen::Quaternionf(in_current_pose_var.rotation.rotation(0), in_current_pose_var.rotation.rotation(1), in_current_pose_var.rotation.rotation(2), in_current_pose_var.rotation.rotation(3));
        Eigen::Quaternionf qBase;
        qBase.w() = in_current_pose_var.rotation.rotation(0);
        qBase.x() = in_current_pose_var.rotation.rotation(1);
        qBase.y() = in_current_pose_var.rotation.rotation(2);
        qBase.z() = in_current_pose_var.rotation.rotation(3);
        q.normalize();
        qBase.normalize();
        // RTT::log(RTT::Error)
        // << "q     = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << RTT::endlog();
        // RTT::log(RTT::Error) << "qBase = " << qBase.w() << ", " << qBase.x() << ", " << qBase.y() << ", " << qBase.z() << RTT::endlog();
        qBase *= q;
        // RTT::log(RTT::Error) << "resul = " << qBase.w() << ", " << qBase.x() << ", " << qBase.y() << ", " << qBase.z() << RTT::endlog();

        out_pose_var.translation.translation(0) = in_current_pose_var.translation.translation(0) + out_6d_var(0);
        out_pose_var.translation.translation(1) = in_current_pose_var.translation.translation(1) + out_6d_var(1);
        out_pose_var.translation.translation(2) = in_current_pose_var.translation.translation(2) + out_6d_var(2);

        if (isCageActive)
        {
            if (out_pose_var.translation.translation(0) < cageMinX)
            {
                out_pose_var.translation.translation(0) = cageMinX;
            }
            else if (out_pose_var.translation.translation(0) > cageMaxX)
            {
                out_pose_var.translation.translation(0) = cageMaxX;
            }

            if (out_pose_var.translation.translation(1) < cageMinY)
            {
                out_pose_var.translation.translation(1) = cageMinY;
            }
            else if (out_pose_var.translation.translation(1) > cageMaxY)
            {
                out_pose_var.translation.translation(1) = cageMaxY;
            }

            if (out_pose_var.translation.translation(2) < cageMinZ)
            {
                out_pose_var.translation.translation(2) = cageMinZ;
            }
            else if (out_pose_var.translation.translation(2) > cageMaxZ)
            {
                out_pose_var.translation.translation(2) = cageMaxZ;
            }
        }

        out_pose_var.rotation.rotation(0) = qBase.w();
        out_pose_var.rotation.rotation(1) = qBase.x();
        out_pose_var.rotation.rotation(2) = qBase.y();
        out_pose_var.rotation.rotation(3) = qBase.z();

        // out_pose_var.rotation.rotation(0) = 0;
        // out_pose_var.rotation.rotation(1) = 0;
        // out_pose_var.rotation.rotation(2) = 1;
        // out_pose_var.rotation.rotation(3) = 0;

        // save state to not return to the initially read pose.
        in_current_pose_var = out_pose_var;
        out_pose_port.write(out_pose_var);
    }
#endif
}

#ifdef USE_RSTRT
void SpaceNavOrocos::resetOrientation(float w, float x, float y, float z)
{
    out_pose_var.rotation.rotation(0) = w;
    out_pose_var.rotation.rotation(1) = x;
    out_pose_var.rotation.rotation(2) = y;
    out_pose_var.rotation.rotation(3) = z;
}

void SpaceNavOrocos::resetPoseToInitial()
{
    in_current_pose_var = initial_pose_var;
}

void SpaceNavOrocos::resetPose(rstrt::geometry::Pose pose)
{
    in_current_pose_var = pose;
}
#endif

void SpaceNavOrocos::stopHook()
{
    RTT::extras::FileDescriptorActivity *activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity)
        activity->clearAllWatches();
    interface->setLedState(0);
}

void SpaceNavOrocos::cleanupHook()
{
    if (interface)
    {
        delete interface;
    }
}

void SpaceNavOrocos::displayStatus()
{
    RTT::log(RTT::Error) << "[" << this->getName() << "] Info\n"
                         << "Listening to interface " << getFileDescriptor() << "\n"
                         << "Button 1 " << (!button1_old ? "Not pressed => Translation enabled" : "Pressed => Translation disabled") << "\n"
                         << "Button 2 " << (!button2_old ? "Not pressed => Orientation enabled" : "Pressed => Orientation disabled") << "\n"
                         << "enableX = " << enableX << "\n"
                         << "enableY = " << enableY << "\n"
                         << "enableZ = " << enableZ << "\n"
                         << "enableA = " << enableA << "\n"
                         << "enableB = " << enableB << "\n"
                         << "enableC = " << enableC << "\n"
                         << RTT::endlog();
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::hw::SpaceNavOrocos)