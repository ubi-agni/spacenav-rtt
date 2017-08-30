 # ============================================================
 #
 # This file is a part of the spacenav-rsb project
 #
 # Copyright (C) 2017 by Dennis Leroy Wigand <dwigand at techfak dot uni-bielefeld dot de>
 #
 # This file may be licensed under the terms of the
 # GNU Lesser General Public License Version 3 (the ``LGPL''),
 # or (at your option) any later version.
 #
 # Software distributed under the License is distributed
 # on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 # express or implied. See the LGPL for the specific language
 # governing rights and limitations.
 #
 # You should have received a copy of the LGPL along with this
 # program. If not, go to http://www.gnu.org/licenses/lgpl.html
 # or write to the Free Software Foundation, Inc.,
 # 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 #
 # The development of this software was supported by:
 #   CoR-Lab, Research Institute for Cognition and Robotics
 #     Bielefeld University
 #
 # ============================================================
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT("spacenav-rsb")

FIND_PACKAGE(RSC 0.10 REQUIRED)
FIND_PACKAGE(RSB 0.10 REQUIRED)
FIND_PACKAGE(RST 0.10 REQUIRED sandbox)
FIND_PACKAGE(rst-converters 0.10 REQUIRED)

INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RSB_INCLUDE_DIRS}
                                  ${RSC_INCLUDE_DIRS}
                                  ${RST_INCLUDE_DIRS}
                                  ${RST_CONVERTERS_INCLUDE_DIRS})

LIST(INSERT CMAKE_MODULE_PATH 0 ${RSC_CMAKE_MODULE_PATH})

ADD_DEFINITIONS(${RST_CONVERTERS_CFLAGS})

INCLUDE(DefineProjectVersion)
INCLUDE(PedanticCompilerWarnings)

DEFINE_PROJECT_VERSION("SPACENAV_RSB_" 0 1 0 "archive")
SET(SO_VERSION "${SPACENAV_RSB_VERSION_MAJOR}.${SPACENAV_RSB_VERSION_MINOR}")

# mark-start::binary-name
SET(BINARY_NAME "spacenav-rsb")
# mark-end::binary-name

ADD_SUBDIRECTORY(src)
