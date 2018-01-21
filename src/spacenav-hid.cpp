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
#include "spacenav-hid.hpp"

#include <linux/input.h>
#include <linux/limits.h>
#include <dirent.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>

#define PATH_BUFFER_SIZE (1024)

#define DEF_MINVAL  (-500)
#define DEF_MAXVAL  500
#define DEF_RANGE (DEF_MAXVAL - DEF_MINVAL)

// #define VENDOR_ID    0x046d
// #define PRODUCT_ID   0xc626
using namespace std;

namespace cosima {

  namespace hw {


SpaceNavHID::SpaceNavHID(){
   oldValues.reset();
}

SpaceNavHID::~SpaceNavHID(){
    close();
}

int SpaceNavHID::getFileDescriptor() {
    return fd;
}

bool SpaceNavHID::init() {
  btn_0_pressed = false;
  btn_1_pressed = false;
  mode = 0;
  struct dirent *entry;
  DIR *dp;
  // char path[PATH_BUFFER_SIZE];
  input_id_td device_info;
  // const char *devDirectory = "/dev/input/";
  std::string devDirectory = "/dev/input/";
  std::string path = devDirectory;
  const char *dev_event_file_name = "/dev/input/spacenavigator";

    // Try backup Symlink first.
    std::cout << "[SpaceNavHID] " << "Searching for device on " << dev_event_file_name << std::endl;
    fd = open(dev_event_file_name, O_RDWR | O_NONBLOCK);
    if ((fd > -1) && checkDeviceId(fd, device_info)) {
        std::cout << "[SpaceNavHID] " << "Using evdev device: " << dev_event_file_name << std::endl;
        ::close(fd);
        mode = determineDeviceMode(dev_event_file_name);
        if (mode == -1) {
            std::cerr << "[SpaceNavHID] " << "No operating modes available for " << dev_event_file_name << std::endl;
            return false;
        }
    } else {
  /* open the directory */
  std::cout << "[SpaceNavHID] " << "Searching for a suitable device in " << devDirectory << std::endl;
  dp = opendir(devDirectory.c_str());
  if(dp == NULL) {
    std::cerr << "[SpaceNavHID] " << "Folder not found " << devDirectory << std::endl;
    return false;
  }
  /* walk the directory (non-recursively) */
  while((entry = readdir(dp))) {
    // strncpy(path, devDirectory, sizeof(path));
    path = devDirectory;
    /* if strlen(devDirectory) > sizeof(path) the path won't be NULL terminated
     * and *bad things* will happen. Therfore, we force NULL termination.
     */
    // path[PATH_BUFFER_SIZE-1] = '\0';
    // strncat(path, entry->d_name, sizeof(path));
    path.append(entry->d_name);

    std::cout << "[SpaceNavHID] " << "Checking " << path << " ... ";
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if(-1 == fd) {
      std::cout << "[SpaceNavHID] " << "Nope, cannot open!" << std::endl;
      continue;
    }
    std::cout << "[SpaceNavHID] " << "Open ... ";

    if (checkDeviceId(fd, device_info)) {
        std::cout << "[SpaceNavHID] " << "Vendor and ID do match! " << std::endl;
        break;
    }
    std::cout << "[SpaceNavHID] " << "Vendor and ID do not match." << std::endl;
    ::close(fd);
    fd = -1;
  }
  closedir(dp);

    if(fd == -1) {
        std::cerr << "[SpaceNavHID] " << "Could not fine a matching device." << std::endl;
        return false;
    }
    // needed for the next checks.
    ::close(fd); 
    mode = determineDeviceMode(dev_event_file_name);
    if (mode == -1) {
        std::cerr << "[SpaceNavHID] " << "No operating modes available for " << dev_event_file_name << std::endl;
        return false;
    }
 }


  /* ###################### NUMBER OF AXIS ###################### */
	num_axes = 6;
  std::cout << "[SpaceNavHID] " << "Checking for available axes (expecting " << num_axes << ") ... ";
	unsigned char evtype_mask[(EV_MAX + 7) / 8];
	if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof evtype_mask), evtype_mask) == 0) {
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
	std::cout << "[SpaceNavHID] " << "found " << num_axes << std::endl;

  if (num_axes < 6) {
    std::cerr << "[SpaceNavHID] " << "Something is wrong with the axes!" << std::endl;
    // TODO ?
  }

  /* ###################### MAX AND MIN VALUES ###################### */
	// if the device is an absolute device, find the minimum and maximum axis values
  absinfo = new input_absinfo_td[num_axes];

	for (int i = 0; i < num_axes; i++) {
		absinfo[i].minimum = DEF_MINVAL;
		absinfo[i].maximum = DEF_MAXVAL;
		absinfo[i].fuzz = 0;
	}

  // translation
  if(ioctl (fd, EVIOCGABS (ABS_X), &(absinfo[0])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis X ABS range: [ " << absinfo[0].minimum << " - " << absinfo[0].maximum << " ] at " << absinfo[0].fuzz << std::endl;
  }
  if(ioctl (fd, EVIOCGABS (ABS_Y), &(absinfo[1])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis Y ABS range: [ " << absinfo[1].minimum << " - " << absinfo[1].maximum << " ] at " << absinfo[1].fuzz << std::endl;
  }
  if(ioctl (fd, EVIOCGABS (ABS_Z), &(absinfo[2])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis Z ABS range: [ " << absinfo[2].minimum << " - " << absinfo[2].maximum << " ] at " << absinfo[2].fuzz << std::endl;
  }
  // rotation
  if(ioctl (fd, EVIOCGABS (ABS_RX), &(absinfo[3])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis RX ABS range: [ " << absinfo[3].minimum << " - " << absinfo[3].maximum << " ] at " << absinfo[3].fuzz << std::endl;
  }
    if(ioctl (fd, EVIOCGABS (ABS_RX), &(absinfo[4])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis RY ABS range: [ " << absinfo[4].minimum << " - " << absinfo[4].maximum << " ] at " << absinfo[4].fuzz << std::endl;
  }
  if(ioctl (fd, EVIOCGABS (ABS_RX), &(absinfo[5])) == 0) {
    std::cout << "[SpaceNavHID] " << "Axis RZ ABS range: [ " << absinfo[5].minimum << " - " << absinfo[5].maximum << " ] at " << absinfo[5].fuzz << std::endl;
  }

  return true;
}

bool SpaceNavHID::checkDeviceId(const int fd, input_id_td &device_info) {
    ioctl(fd, EVIOCGID, &device_info);        // get device ID
    if ( //http://spacemice.org/index.php?title=Dev
    ((device_info.vendor == 0x046d) && // Logitech's Vendor ID, used by 3DConnexion until they got their own.
        ( //(ID.product == 0xc603) || // SpaceMouse (untested)
                //(ID.product == 0xc605) || // CADMan (untested)
                //(ID.product == 0xc606) || // SpaceMouse Classic (untested)
                //(ID.product == 0xc621) || // SpaceBall 5000
                //(ID.product == 0xc623) || // SpaceTraveler (untested)
                //(ID.product == 0xc625) || // SpacePilot (untested)
            (device_info.product == 0xc626) || // SpaceNavigators
                //(ID.product == 0xc627) || // SpaceExplorer (untested)
                //(ID.product == 0xc628) || // SpaceNavigator for Notebooks (untested)
                //(ID.product == 0xc629) || // SpacePilot Pro (untested)
                //(ID.product == 0xc62b) || // SpaceMousePro
            0)
        )
    || ((device_info.vendor == 0x256F) && // 3Dconnexion's Vendor ID
        (
            (device_info.product == 0xc62E) || // SpaceMouse Wireless (cable) (untested)
            (device_info.product == 0xc62F) || // SpaceMouse Wireless (receiver) (untested)
            (device_info.product == 0xc631) || // Spacemouse Wireless (untested)
            (device_info.product == 0xc632) || // SpacemousePro Wireless (untested)
            0)
        )
    ) {
        return true;
    }
    return false;
}

void SpaceNavHID::close() {
  if(fd > 0) {
    ::close(fd);
  }
  fd = -1;
}

int SpaceNavHID::determineDeviceMode(const char* device_path) {
    int device_fd = -1;
    if ((device_fd = open(device_path, O_RDWR)) == -1) {
        if ((device_fd = open(device_path, O_RDONLY)) == -1) {
            perror("opening the file you specified");
            return -1;
        }
        fprintf(stderr, "opened device read-only, LEDs won't work\n");
        return 0;
    }

    if (device_fd == -1) {
        std::cerr << "[SpaceNavHID] " << "Error while opening the device at " << device_path << std::endl;
        return -1;
    }
    
    return 1;
}

void SpaceNavHID::getValue(SpaceNavValues &coordinates, SpaceNavValues &rawValues) {
    rawValues = oldValues;
  /* If input events don't come in fast enough a certain DoF may not be 
   * updated during a frame. This results in choppy and ugly animation.
   * To solve this we record the number of frames a certain DoF was idle
   * and only set the DoF to 0 if we reach a certain idleThreshold.
   * When there is activity on a axis the idleFrameCount is reset to 0.
   */
  int i, eventCnt;
  /* how many bytes were read */
  size_t bytesRead;
  /* the events (up to 64 at once) */
  struct input_event events[64];
  /* keep track of idle frames for each DoF for smoother animation. see above */
  static int idleFrameCount[6] = {0, 0, 0, 0, 0, 0};
  // int idleThreshold = 3;

  /* read the raw event data from the device */
  bytesRead = read(fd, events, sizeof(struct input_event) * 64);
  eventCnt = (int) ((long)bytesRead / (long)sizeof(struct input_event));
  if (bytesRead < (int) sizeof(struct input_event)) {
    perror("evtest: short read");
    return;
  }

  /* Increase all idle counts. They are later reset if there is activity */
  for(i = 0; i < 6; ++i) {
    ++idleFrameCount[i];
  }

  /* handle input events sequentially */
  for(i = 0; i < eventCnt; ++i) {

    if (events[i].type == EV_SYN || events[i].type == EV_MSC)
			continue; // ignore EV_MSC and EV_SYN events

    switch (events[i].type) {
    case EV_REL:
          // should not occur...
    case EV_ABS: {
        int axisIndex = events[i].code - ABS_X; // REL_X;
        switch(axisIndex) {
        // case ABS_X: //Same value as REL_* so because of the check above, this is not needed
        case 0:
          rawValues.tx = events[i].value;
          idleFrameCount[0] = 0;
          break;
        //case ABS_Y:
        case 1:
          rawValues.ty = events[i].value;
          idleFrameCount[1] = 0;
          break;
        //case ABS_Z:
        case 2:
          rawValues.tz = events[i].value;
          idleFrameCount[2] = 0;
          break;
        //case ABS_RX:
        case 3:
          rawValues.rx = events[i].value;
          idleFrameCount[3] = 0;
          break;
        //case ABS_RY:
        case 4:
          rawValues.ry = events[i].value;
          idleFrameCount[4] = 0;
          break;
        //case ABS_RZ:
        case 5:
          rawValues.rz = events[i].value;
          idleFrameCount[5] = 0;
          break;

        default:
				  break;
        }
        break;
      }
      case EV_KEY: {
        int btn_index = events[i].code - BTN_0;
        // switch
        if (btn_index == 0) {
          rawValues.button1 = events[i].value;
          if (events[i].value == 1) {
            if (btn_0_pressed) {
              btn_0_pressed = false;
              coordinates.button1 = 0;
              setLedState(0);
            } else {
              btn_0_pressed = true;
              coordinates.button1 = 1;
              setLedState(1);
            }
          }
          // Dead-man switch / trigger
        } else if (btn_index == 1) {
          rawValues.button2 = events[i].value;
          if (events[i].value == 1) {
            if (btn_1_pressed) {
              btn_1_pressed = false;
              coordinates.button2 = 0;
              //setLedState(0);
            } else {
              btn_1_pressed = true;
              coordinates.button2 = 1;
              //setLedState(1);
            }
          }
        }
        break;
      }
      default:
        break;
    }
  }

  // /* Set rawValue to zero if DoF was idle for more than idleThreshold frames */
  // for(i = 0; i < 6; ++i) {
  //   if(idleFrameCount[i] >= idleThreshold) {
  //     if(0==i) {
  //       rawValues.tx = 0;
  //     } else if (1==i) {
  //       rawValues.ty = 0;
  //     } else if (2==i) {
  //       rawValues.tz = 0;
  //     } else if (3==i) {
  //       rawValues.rx = 0;
  //     } else if (4==i) {
  //       rawValues.ry = 0;
  //     } else if (5==i) {
  //       rawValues.rz = 0;
  //     }
  //   }
  // }

  // translation
  // if (!btn_1_pressed) {
    coordinates.tx = -1 * getSlopedOutput(1, rawValues.tx);
    coordinates.ty = -1 * getSlopedOutput(0, rawValues.ty);
    coordinates.tz = -1 * getSlopedOutput(2, rawValues.tz);
  // } else {
  //   coordinates.tx = 0;
  //   coordinates.ty = 0;
  //   coordinates.tz = 0;
  // }
  // rotation
  // if (!btn_0_pressed) {
    coordinates.rx = -1 * getSlopedOutput(3, rawValues.rx);
    coordinates.ry = -1 * getSlopedOutput(4, rawValues.ry);
    coordinates.rz = -1 * getSlopedOutput(5, rawValues.rz);
  // } else {
  //   coordinates.rx = 0;
  //   coordinates.ry = 0;
  //   coordinates.rz = 0;
  // }
  oldValues = rawValues;
}

double SpaceNavHID::getSlopedOutput(const int axisIndex, const double value) {
      double output_end = 500;
			double output_start = -500;
			double input_end = absinfo[axisIndex].maximum;
			double input_start = absinfo[axisIndex].minimum;
			double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
			return output_start + floor((slope * (value - input_start)) + 0.5);
}

bool SpaceNavHID::setLedState(const int state) {
  if (fd == -1) {
    return false;
  }
	struct input_event evLed;
	memset(&evLed, 0, sizeof evLed);
	evLed.type = EV_LED;
	evLed.code = LED_MISC;
	evLed.value = state; // on 1 / off 0

	if (write(fd, &evLed, sizeof evLed) == -1) {
		// fprintf(stderr, "failed to turn LED %s\n", state ? "on" : "off");
		return false;
	}
	return true;
}

int SpaceNavHID::getNumAxes() {
  return num_axes;
}

}

}