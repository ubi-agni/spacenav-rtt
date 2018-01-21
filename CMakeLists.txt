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
project("spacenav")

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fPIC")

FIND_PACKAGE(RSC 0.16 REQUIRED)
FIND_PACKAGE(RSB 0.16 REQUIRED)
FIND_PACKAGE(RST 0.16 REQUIRED sandbox)
FIND_PACKAGE(rst-converters 0.16 REQUIRED)
LIST(INSERT CMAKE_MODULE_PATH 0 ${RSC_CMAKE_MODULE_PATH})
ADD_DEFINITIONS(${RST_CONVERTERS_CFLAGS})

INCLUDE(DefineProjectVersion)
INCLUDE(PedanticCompilerWarnings)
DEFINE_PROJECT_VERSION("SPACENAV_RSB_" 0 1 0 "archive")
SET(SO_VERSION "${SPACENAV_RSB_VERSION_MAJOR}.${SPACENAV_RSB_VERSION_MINOR}")
SET(BINARY_NAME "spacenav-rsb")

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
include_directories(${USE_OROCOS_INCLUDE_DIRS})

#set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
#find_package(Boost COMPONENTS thread REQUIRED)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

include_directories(BEFORE SYSTEM src
    src/orocos
    #${Boost_INCLUDE_DIR}
    #${Eigen_INCLUDE_DIRS}
    ${RSB_INCLUDE_DIRS}
    ${RSC_INCLUDE_DIRS}
    ${RST_INCLUDE_DIRS}
    ${RST_CONVERTERS_INCLUDE_DIRS}
)

ADD_LIBRARY(spacenav-hid SHARED
  src/spacenav-hid.cpp
)

#ADD_SUBDIRECTORY(src) # START
ADD_EXECUTABLE(${BINARY_NAME} "src/spacenav-rsb.cpp")
TARGET_LINK_LIBRARIES(${BINARY_NAME} ${RSB_LIBRARIES} ${RST_CONVERTERS_LIBRARIES})

INSTALL(TARGETS ${BINARY_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
#ADD_SUBDIRECTORY(src) # END

ADD_EXECUTABLE(spacenav-hid-test "src/spacenav-hid-test.cpp")
TARGET_LINK_LIBRARIES(spacenav-hid-test spacenav-hid)


orocos_component(spacenav-orocos "src/orocos/spacenav-orocos.cpp")

set_target_properties(spacenav-orocos PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(spacenav-orocos
   ${USE_OROCOS_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   #${Boost_LIBRARIES}
   #${RST-RT_LIBRARIES}
   spacenav-hid
)

orocos_generate_package()
