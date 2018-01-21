/* ============================================================
 *
 * This file is a part of the spacenav-rsb project
 *
 * Copyright (C) 2017 by Dennis Leroy Wigand <dwigand at techfak dot uni-bielefeld dot de>
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

//rsb0.14 logger -s detailed --on-error=continue --idl-path "/home/dwigand/citk/systems/cogimon-minimal-nightly/share/rst0.14/proto/stable" --idl-path "/home/dwigand/citk/systems/cogimon-minimal-nightly/share/rst0.14/proto/sandbox" --load-idl '/home/dwigand/citk/systems/cogimon-minimal-nightly/share/rst0.14/proto/**/*.proto' /
//./src/spacenav-rsb /dev/input/event21
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h>
#include <linux/limits.h>
#include <linux/input.h>
#include <signal.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>

#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/Factory.h>
#include <rsb/Listener.h>
#include <rsb/converter/Repository.h>
#include <rsb/Informer.h>

#include <rsb/converter/ProtocolBufferConverter.h>
#include <rst/dynamics/Wrench.pb.h>
#include <rst/dynamics/Forces.pb.h>
#include <rst/dynamics/Torques.pb.h>

#define DEF_MINVAL  (-500)
#define DEF_MAXVAL  500
#define DEF_RANGE (DEF_MAXVAL - DEF_MINVAL)

using namespace std;
namespace po = boost::program_options;

#define VENDOR_ID    0x046d
#define PRODUCT_ID   0xc626

// mark-start::commandline-arguments
string inScope = "/video";
string outScope = "/device/spacenav/output/wrench";

// std::string open_device (std::string device_vid, std::string device_pid)
// {       
//     try
//     {
//         std::ifstream file_input;
//         std::size_t pos;
//         std::string device_path, current_line, search_str, event_str;
//         std::string device_list_file = "/proc/bus/input/devices";
//         bool vid_pid_found = false;
//         bool debug = true;

//         // 1. open device list file
//         file_input.open(device_list_file.c_str());
//         if (!file_input.is_open())
//         {
//             std::cerr << "file_input.open >> " << std::strerror(errno) << std::endl;
//             throw -2;
//         }

//         // 2. search for first VID:PID and get event number
//         search_str = "Vendor=" + device_vid + " Product=" + device_pid;
//         while (getline(file_input, current_line))
//         {
//             if (!vid_pid_found)
//             {
//                 pos = current_line.find(search_str, 0);
//                 if (pos != std::string::npos)
//                 {
//                     vid_pid_found = true;
//                     search_str = "event";
//                 }               
//             }
//             else
//             {
//                 pos = current_line.find(search_str, 0);
//                 if (pos != std::string::npos)
//                 {
//                     event_str = current_line.substr(pos);
//                     std::size_t found_first_space = event_str.find_first_of(" ");
//                     std::size_t found_first_event = event_str.find_first_of("event");
//                     if ((found_first_space > 0)) {
//                       std::string result = event_str.substr (found_first_event,found_first_space);
//                       event_str = result;
//                     }
//                     break;
//                 }
//             }
//         }

//         // 3.  build device path
//         device_path = "/dev/input/" + event_str;
//         if (debug) std::cout << "device_path = " << device_path << std::endl;   
//         // 4.  connect to device
//         return device_path;
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "e.what() = " << e.what() << std::endl;
//         throw -1;
//     }

//     return "";
// }

void error(char *msg) {
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(EXIT_FAILURE);
}

bool setLedState(int device_fd, int state) {
// ###################### SET STATE OF LED ######################
	struct input_event evLed;
	memset(&evLed, 0, sizeof evLed);
	evLed.type = EV_LED;
	evLed.code = LED_MISC;
	evLed.value = state; // on 1 / off 0

	if (write(device_fd, &evLed, sizeof evLed) == -1) {
		fprintf(stderr, "failed to turn LED %s\n", state ? "on" : "off");
		return false;
	}
	return true;
// ###################### SET STATE OF LED ######################
}

void handleCommandline(int argc, char *argv[]) {

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")("inscope,i",
			po::value<string>(&inScope), "Scope for receiving input images.")(
			"outscope,o", po::value<string>(&outScope),
			"Scope for sending the results.");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(
			po::command_line_parser(argc, argv).options(options).positional(p).run(),
			vm);

	// first, process the help option
	if (vm.count("help")) {
		cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

}
// mark-end::commandline-arguments

int main(int argc, char *argv[]) {

	int device_fd = -1;

	// Backup Symlink!
	std::string dev_event_file_name = "/dev/input/spacenavigator";

	// try to find 3DConnexion SpaceNavigator first
	int i = 0;
	struct input_id ID;

	while (i < 32) {
		dev_event_file_name = "/dev/input/event" + boost::lexical_cast<std::string>(i++);
		device_fd = open(dev_event_file_name.c_str(), O_RDWR | O_NONBLOCK);
		if (device_fd > 0) {
			ioctl(device_fd, EVIOCGID, &ID);        // get device ID
			if ( //http://spacemice.org/index.php?title=Dev
			((ID.vendor == 0x046d) && // Logitech's Vendor ID, used by 3DConnexion until they got their own.
				( //(ID.product == 0xc603) || // SpaceMouse (untested)
					  //(ID.product == 0xc605) || // CADMan (untested)
					  //(ID.product == 0xc606) || // SpaceMouse Classic (untested)
					  //(ID.product == 0xc621) || // SpaceBall 5000
					  //(ID.product == 0xc623) || // SpaceTraveler (untested)
					  //(ID.product == 0xc625) || // SpacePilot (untested)
					(ID.product == 0xc626) || // SpaceNavigators
					  //(ID.product == 0xc627) || // SpaceExplorer (untested)
					  //(ID.product == 0xc628) || // SpaceNavigator for Notebooks (untested)
					  //(ID.product == 0xc629) || // SpacePilot Pro (untested)
					  //(ID.product == 0xc62b) || // SpaceMousePro
					0)
				)
			|| ((ID.vendor == 0x256F) && // 3Dconnexion's Vendor ID
				(
					(ID.product == 0xc62E) || // SpaceMouse Wireless (cable) (untested)
					(ID.product == 0xc62F) || // SpaceMouse Wireless (receiver) (untested)
					(ID.product == 0xc631) || // Spacemouse Wireless (untested)
					(ID.product == 0xc632) || // SpacemousePro Wireless (untested)
					0)
				)
			) {
				printf("Using evdev device: %s\n", dev_event_file_name.c_str());
				break;
			} else {
				close(device_fd);
				device_fd = -1;
			}
		}
	}

	rsb::converter::Converter<string>::Ptr poseConverter(
			new rsb::converter::ProtocolBufferConverter<rst::dynamics::Wrench>());
	rsb::converter::converterRepository<string>()->registerConverter(
			poseConverter);

	rsb::Factory& factory = rsb::getFactory();

	rsb::Informer<rst::dynamics::Wrench>::Ptr informer = factory.createInformer<
			rst::dynamics::Wrench>(rsb::Scope(outScope));

	if ((device_fd = open(dev_event_file_name.c_str(), O_RDWR)) == -1) {
		if ((device_fd = open(dev_event_file_name.c_str(), O_RDONLY)) == -1) {
			perror("opening the file you specified");
			exit(EXIT_FAILURE);
		}
		fprintf(stderr, "opened device read-only, LEDs won't work\n");
	}

	if(device_fd <= 0) {
		std::cerr << "Error while opening the device at " << dev_event_file_name << std::endl;
		exit(EXIT_FAILURE);
	}

// ###################### NUMBER OF AXIS ######################
	int num_axes = 6;
	unsigned char evtype_mask[(EV_MAX + 7) / 8];
	if (ioctl(device_fd, EVIOCGBIT(EV_ABS, sizeof evtype_mask), evtype_mask)
			== 0) {
		num_axes = 0;
		for (int i = 0; i < ABS_CNT; i++) {
			int idx = i / 8;
			int bit = i % 8;

			if (evtype_mask[idx] & (1 << bit)) {
				num_axes++;
			} else {
				break;
			}
		}
	}
	printf("  Number of axes: %d\n", num_axes);
// ###################### NUMBER OF AXIS ######################

// ###################### MAX AND MIN VALUES ######################
	// if the device is an absolute device, find the minimum and maximum axis values
	int* minval = (int*) malloc(num_axes * sizeof(int));
	int* maxval = (int*) malloc(num_axes * sizeof(int));
	int* fuzz = (int*) malloc(num_axes * sizeof(int));

	struct input_absinfo absinfo;
	for (int i = 0; i < num_axes; i++) {
		minval[i] = DEF_MINVAL;
		maxval[i] = DEF_MAXVAL;
		fuzz[i] = 0;

		if (ioctl(device_fd, EVIOCGABS(i), &absinfo) == 0) {
			minval[i] = absinfo.minimum;
			maxval[i] = absinfo.maximum;
			fuzz[i] = absinfo.fuzz;

			printf("  Axis %d value range: %d - %d (fuzz: %d)\n", i, minval[i],
					maxval[i], fuzz[i]);
		}
	}
// ###################### MAX AND MIN VALUES ######################

	/* initialize the input buffer */

	//struct input_event ev;
	struct input_event event_data;  // = &ev;

	bool btn_0_pressed = false;
	bool btn_1_pressed = false;

	boost::shared_ptr<rst::dynamics::Wrench> outWrench(
			new rst::dynamics::Wrench());

	rst::dynamics::Forces* wrenchForces = outWrench->mutable_forces();
	wrenchForces->set_x(0.0);
	wrenchForces->set_y(0.0);
	wrenchForces->set_z(0.0);

	rst::dynamics::Torques* wrenchTorques = outWrench->mutable_torques();
	wrenchTorques->set_a(0.0);
	wrenchTorques->set_b(0.0);
	wrenchTorques->set_c(0.0);

	while (true) {
		ssize_t num_read = read(device_fd, &event_data, sizeof(event_data));

		if (sizeof(event_data) != num_read) {
			fputs("read failed\n", stderr);
			exit(EXIT_FAILURE);
		}

		if (event_data.type == EV_SYN || event_data.type == EV_MSC)
			continue; // ignore EV_MSC and EV_SYN events

		// memcpy(sock_buffer, event_data, sizeof(ev));
		// int num_sent = sendto(
		//   sock_fd,
		//   sock_buffer,
		//   sock_buffer_size,
		//   0,
		//   (struct sockaddr *) &sock_addr,
		//   sizeof(sock_addr)
		// );

		// if (num_sent < 0) {
		//   perror("sending to socket");
		// }

		switch (event_data.type) {
		case EV_REL:
			// inp->type = INP_MOTION;
			// inp->idx = iev.code - REL_X;
			// inp->val = iev.value;
			printf("EV_REL(%d): %d\n", event_data.code - REL_X,
					event_data.value);
			break;

		case EV_ABS: {
			// inp->type = INP_MOTION;
			// inp->idx = iev.code - ABS_X;
			// inp->val = map_range(dev, inp->idx, iev.value);
			/*printf("[%s] EV_ABS(%d): %d (orig: %d)\n", dev->name, inp->idx, inp->val, iev.value);*/

// make sure that all the signs are also properly assigned!
			int a = event_data.code - ABS_X;
			double output_end = 500;
			double output_start = -500;
			double input_end = maxval[a];
			double input_start = minval[a];
			double slope = 1.0 * (output_end - output_start)
					/ (input_end - input_start);
			double output = output_start
					+ floor((slope * (event_data.value - input_start)) + 0.5);

			printf("EV_ABS(%d): %f\n", a, output);

			switch (a) {
			case 0:
				if (!btn_1_pressed)
					wrenchForces->set_y(-1 * output);
				break;
			case 1:
				if (!btn_1_pressed)
					wrenchForces->set_x(-1 * output);
				break;
			case 2:
				if (!btn_1_pressed)
					wrenchForces->set_z(-1 * output);
				break;
			case 3:
				if (!btn_0_pressed)
					wrenchTorques->set_a(-1 * output);
				break;
			case 4:
				if (!btn_0_pressed)
					wrenchTorques->set_b(output);
				break;
			case 5:
				if (!btn_0_pressed)
					wrenchTorques->set_c(-1 * output);
				break;
			default:
				break;
			}

			informer->publish(outWrench);

			// int range = maxval[a] - minval[a];
			// if(range <= 0) {
			//   printf("EV_ABS(%d): %d\n", event_data.code - ABS_X, event_data.value);
			// } else {
			//   printf("yayyyyyyyyyyy EV_ABS(%d): %d\n", a, (event_data.value - minval[a]) * DEF_RANGE / range + DEF_MINVAL);
			// }
			break;
		}
		case EV_KEY: {
			// inp->type = INP_BUTTON;
			// inp->idx = iev.code - BTN_0;
			// inp->val = iev.value;
			int btn_index = event_data.code - BTN_0;
			// switch
			if (btn_index == 0) {
				if (event_data.value == 1) {
					if (btn_0_pressed) {
						btn_0_pressed = false;
						setLedState(device_fd, 0);
					} else {
						btn_0_pressed = true;
						setLedState(device_fd, 1);
					}
				}      // else {
					   //btn_0_pressed = false;
					   //setLedState(device_fd, 0);
				//}
				// Dead-man switch / trigger
			} else if (btn_index == 1) {
				if (event_data.value == 1) {
					//btn_1_pressed = true;
					if (btn_1_pressed) {
						btn_1_pressed = false;
						//setLedState(device_fd, 0);
					} else {
						btn_1_pressed = true;
						//setLedState(device_fd, 1);
					}
				}            // else {
							 //     btn_1_pressed = false;
							 // }
			}
			printf("EV_KEY(%d): %d\n", event_data.code - BTN_0,
					event_data.value);
			break;
		}
		case EV_SYN:
			// inp->type = INP_FLUSH;
			/*printf("[%s] EV_SYN\n", dev->name);*/
			printf("EV_SYN\n");
			break;

		default:
			printf("unhandled event: %d\n", event_data.type);
		}

		/*
		 (5): - _____________ (5): +
		 /             \
           1(3): -
		 2 : - ^
		 \ |
		 0(4):-(+) <__\|__> 0(4):+(-)
		 |\
               | \
               v  2 : +
		 1(3): +
		 3DConnexion
		 */

		// #ifdef DEBUG
		//   fprintf(
		//     stderr,
		//     "sent %d bytes from %s. type: %d code: %d value: %d\n",
		//     num_sent, DEVICE_ID, event_data.type, event_data.code, event_data.value
		//  );
		// #endif
	}
	return EXIT_SUCCESS;
}
