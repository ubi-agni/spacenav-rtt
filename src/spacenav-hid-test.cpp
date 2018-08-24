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
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace cosima::hw;

int main(int argc, char **argv)
{

    if (argc != 1)
    {
        exit(0);
    }

    SpaceNavHID *c = new SpaceNavHID();
    c->initDevice();
    SpaceNavValues c1;
    SpaceNavValues c2;

    while (1)
    {
        c->getValue(c1, c2);
        std::cout << ">> x = " << c1.tx << ", y = " << c1.ty << ", z = " << c1.tz << ", rx = " << c1.rx << ", ry = " << c1.ry << ", rz = " << c1.rz << ", b0 = " << c1.button1 << ", b1 = " << c1.button2 << std::endl;
        usleep(4000);
    }
}