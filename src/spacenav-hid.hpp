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

/*
spacenavd - a free software replacement driver for 6dof space-mice.
Copyright (C) 2007-2018 John Tsiombikas <nuclear@member.fsf.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
// Code from https://github.com/FreeSpacenav/spacenavd/blob/master/src/dev_usb_linux.c

#ifndef _COSIMA_SpaceNavHID_H_
#define _COSIMA_SpaceNavHID_H_

typedef struct input_id input_id_td;
typedef struct input_absinfo input_absinfo_td;

namespace cosima
{

namespace hw
{

class SpaceNavValues
{
public:
  double tx;
  double ty;
  double tz;
  double rx;
  double ry;
  double rz;
  int button1;
  int button2;

  SpaceNavValues()
  {
    reset();
  }

  // ~SpaceNavValues();

  void reset()
  {
    tx = 0;
    ty = 0;
    tz = 0;
    rx = 0;
    rz = 0;
    button1 = false;
    button2 = false;
  }
};

class SpaceNavHID
{

public:
  SpaceNavHID();
  ~SpaceNavHID();

  bool init();

  void close();

  int getFileDescriptor();

  void getValue(SpaceNavValues &coordiantes, SpaceNavValues &rawValues);

  bool setLedState(const int state);

  int getNumAxes();

protected:
  int fd;
  int mode;
  int num_axes;
  SpaceNavValues oldValues;
  bool btn_0_pressed;
  bool btn_1_pressed;

private:
  bool checkDeviceId(const int fd, input_id_td &device_info);

  /**
     * Determines whether the device can use the LEDs (write mode == 1) or not (read mode only == 0).
     * If an error occurs the return value will be -1.
     */
  int determineDeviceMode(const char *device_path);

  double getSlopedOutput(const int axisIndex, const double value);

  input_absinfo_td *absinfo;
};

}; // namespace hw

}; // namespace cosima

#endif