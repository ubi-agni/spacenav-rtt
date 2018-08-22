 # ============================================================
 #
 # This file is a part of the SpaceNav (CoSiMA) project
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
cmake_minimum_required(VERSION 2.8.3)
project(spacenav)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fPIC")

find_package(OROCOS-RTT)
if (OROCOS-RTT_FOUND)
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  include_directories(${USE_OROCOS_INCLUDE_DIRS})
  message(STATUS "OROCOS-RTT found!")
  message(STATUS "USE_OROCOS_INCLUDE_DIRS: ${USE_OROCOS_INCLUDE_DIRS}")
  message(STATUS "USE_OROCOS_LIBRARIES: ${USE_OROCOS_LIBRARIES}")
  message(STATUS "OROCOS-RTT_LIBRARIES: ${OROCOS-RTT_LIBRARIES}")

  find_package(RST-RT REQUIRED)
  message(STATUS "RST-RT version: ${RST-RT_VERSION}")
  message(STATUS "RST-RT_INCLUDE_DIRS: ${RST-RT_INCLUDE_DIRS}")
  message(STATUS "RST-RT_LIBRARY_DIRS: ${RST-RT_LIBRARY_DIRS}")
  message(STATUS "RST-RT_LIBRARY_DIR: ${RST-RT_LIBRARY_DIR}")
  message(STATUS "RST-RT_LIBRARIES: ${RST-RT_LIBRARIES}")
  include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
  add_definitions(${RST-RT_CFLAGS})
  link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})
endif()

#set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
#find_package(Boost COMPONENTS thread REQUIRED)

SET(LIBRARY_NAME "spacenav-hid")
include_directories(BEFORE SYSTEM src
    #${Boost_INCLUDE_DIR}
    #${Eigen_INCLUDE_DIRS}
)

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  src/spacenav-hid.cpp
)

ADD_EXECUTABLE(${LIBRARY_NAME}-test "src/spacenav-hid-test.cpp")
TARGET_LINK_LIBRARIES(${LIBRARY_NAME}-test ${LIBRARY_NAME})

if (OROCOS-RTT_FOUND)
  message(STATUS "######################################################")
  message(STATUS "### Compiling OROCOS-RTT wrapper for SpaceNav HID! ###")
  message(STATUS "######################################################")
  SET(BINARY_NAME_OROCOS "spacenav-orocos")
  include_directories(BEFORE SYSTEM
    src/orocos
  )
  orocos_component(spacenav-orocos "src/orocos/spacenav-orocos.cpp")

  set_target_properties(${BINARY_NAME_OROCOS} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)

  target_link_libraries(${BINARY_NAME_OROCOS}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${RST-RT_LIBRARIES}
    ${LIBRARY_NAME}
  )
  orocos_generate_package()
endif()