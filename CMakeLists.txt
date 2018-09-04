 # ============================================================
 #
 # This file is a part of the SpaceNav (CoSiMA) project
 #
 # Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at techfak dot uni-bielefeld dot de>
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

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Modules/)

find_package(OROCOS-RTT)
if (OROCOS-RTT_FOUND)
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  include_directories(${USE_OROCOS_INCLUDE_DIRS})
  message(STATUS "OROCOS-RTT found!")
  message(STATUS "USE_OROCOS_INCLUDE_DIRS: ${USE_OROCOS_INCLUDE_DIRS}")
  message(STATUS "USE_OROCOS_LIBRARIES: ${USE_OROCOS_LIBRARIES}")
  message(STATUS "OROCOS-RTT_LIBRARIES: ${OROCOS-RTT_LIBRARIES}")

  find_package(RST-RT QUIET)
  # message(STATUS "RST-RT found: ${RST-RT_FOUND}")
  # message(STATUS "RST-RT version: ${RST-RT_VERSION}")
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

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  message(STATUS "Checking for xenomai")
  find_package(Xenomai REQUIRED)
  #message(STATUS ${Xenomai_LIBRARY_DIRS})
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS XENOMAI)
  # Allows to build for Xenomai
  if(XENOMAI_FOUND OR (${OROCOS_TARGET} STREQUAL "xenomai"))
      message(STATUS "######################################################")
      message(STATUS "###    Compiling for Xenomai ${XENOMAI_VERSION}!")
      message(STATUS "###    with:")
      message(STATUS "###         XENOMAI_RTDM_LIBRARIES   : ${XENOMAI_RTDM_LIBRARIES}")
      message(STATUS "###         XENOMAI_RTDM_INCLUDE_DIRS: ${XENOMAI_RTDM_INCLUDE_DIRS}")
      message(STATUS "###         XENOMAI_RTDM_CFLAGS      : ${XENOMAI_RTDM_CFLAGS}")
      message(STATUS "######################################################")

      # In Xenomai 3 it contains RTnet
      include_directories(${XENOMAI_RTDM_INCLUDE_DIRS})
      add_definitions(${XENOMAI_RTDM_CFLAGS})

      # In Xenomai 2 we need the external RTnet
      # if(${XENOMAI_VERSION_MAJOR} EQUAL 2)
      #     find_package(RTnet REQUIRED)
      #     include_directories(${RTNET_INCLUDE_DIRS})
      # endif()

      set_target_properties(${LIBRARY_NAME} PROPERTIES COMPILE_FLAGS -DXENOMAI_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
      target_link_libraries(${LIBRARY_NAME}
        ${XENOMAI_RTDM_LIBRARIES}
      )
  endif()
endif()
ADD_EXECUTABLE(${LIBRARY_NAME}-test "src/spacenav-hid-test.cpp")
TARGET_LINK_LIBRARIES(${LIBRARY_NAME}-test ${LIBRARY_NAME})

if (OROCOS-RTT_FOUND)
  message(STATUS "######################################################")
  message(STATUS "### Compiling OROCOS-RTT wrapper for SpaceNav HID!")
  message(STATUS "######################################################")
  SET(BINARY_NAME_OROCOS "spacenav-orocos")
  include_directories(BEFORE SYSTEM
    src/orocos
  )
  orocos_component(spacenav-orocos "src/orocos/spacenav-orocos.cpp")

  set_target_properties(${BINARY_NAME_OROCOS} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
  if (RST-RT_FOUND)
    message(STATUS "######################################################")
    message(STATUS "###             Using RSTRT Data Types!")
    message(STATUS "######################################################")
    
    target_compile_definitions(${BINARY_NAME_OROCOS} PRIVATE USE_RSTRT)
    target_link_libraries(${BINARY_NAME_OROCOS}
      ${USE_OROCOS_LIBRARIES}
      ${OROCOS-RTT_LIBRARIES}
      ${RST-RT_LIBRARIES}
      ${LIBRARY_NAME}
    )
  else()
    target_link_libraries(${BINARY_NAME_OROCOS}
      ${USE_OROCOS_LIBRARIES}
      ${OROCOS-RTT_LIBRARIES}
      ${LIBRARY_NAME}
    )
  endif()

  orocos_generate_package()
endif()