#include "spacenav-orocos.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp>

using namespace cosima::hw;

SpaceNavOrocos::SpaceNavOrocos(std::string const& name) : RTT::TaskContext(name) {
    interface = new SpaceNavHID();
}

// SpaceNavOrocos::~SpaceNavOrocos() {
//     if (interface) {
//         delete interface;
//     }
// }

bool SpaceNavOrocos::configureHook() {
    if (!interface->init()) {
        RTT::log(RTT::Error) << "[" << this->getName() << "] " << "Unable to access Space Nav at " << getFileDescriptor() << RTT::endlog();
	    return false;
    }

    if (this->getPort("")) {
        this->ports()->removePort("");
    }
    out_6d_var = Eigen::VectorXf(interface->getNumAxes());
    out_6d_var.setZero();
    out_6d_port.setName("out_6d_port");
    out_6d_port.doc("Output port for 6D command vector");
    out_6d_port.setDataSample(out_6d_var);
    ports()->addPort(out_6d_port);

    values.reset();
    rawValues.reset();

    // indicate proper setup by flashing the led!
    for (uint i = 0; i < 3; i++) {
        interface->setLedState(1);
        usleep(100000);
        interface->setLedState(0);
        usleep(100000);
    }

    return true;
}

int SpaceNavOrocos::getFileDescriptor() {
    return interface->getFileDescriptor();
}

bool SpaceNavOrocos::startHook() {
    RTT::extras::FileDescriptorActivity* activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity) {
        activity->watch(getFileDescriptor());
        // get trigger a least every 25 ms
        activity->setTimeout(25);
    }
    return true;
}

// void SpaceNavOrocos::errorHook() {
//     RTT::log(RTT::Fatal) << "ERROR??? TODO" << RTT::endlog();
// }

void SpaceNavOrocos::updateHook() {
    interface->getValue(values, rawValues);

    // TODO do some scaling!

    if (!values.button1) {
        out_6d_var(0) = values.tx;
        out_6d_var(1) = values.ty;
        out_6d_var(2) = values.tz;
    }

    if (!values.button2) {
        out_6d_var(3) = values.rx;
        out_6d_var(4) = values.ry;
        out_6d_var(5) = values.rz;
    }

    // rcmd.buttonValue.push_back(values.button1);
    // rcmd.buttonValue.push_back(values.button2);

    out_6d_port.write(out_6d_var);

    // if (interface->getNumAxes() != axisScales.size()) {
    //     // TODO send zero or just do not scale ??
    //     return;
    // }
    
    // for(size_t i = 0 ; i < axisScales.size(); i++)
    // {
    //     rscaled.axisValue[i] *= axisScales[i];
    // }
}


void SpaceNavOrocos::stopHook() {
    RTT::extras::FileDescriptorActivity* activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if(activity)
        activity->clearAllWatches();
}

void SpaceNavOrocos::cleanupHook() {
    if (interface) {
        delete interface;
    }
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(cosima::hw::SpaceNavOrocos)