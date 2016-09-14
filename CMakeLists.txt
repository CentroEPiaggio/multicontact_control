# 
#  Copyright (C) 2016 Walkman
#  Author: Alessandro Settimi, Mirko Ferrati, Danilo Caporale, Edoardo Farnioli
#  email: ale.settimi@gmail.com
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
project(muticontact_control)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE RELEASE)

# #stuff to provide a shared library to the whole superbuild
# include(ExternalProject)
# 
# set(VARS_PREFIX "ARMS_IK")
# find_package(YCM REQUIRED)
# 
# include(YCMDefaultDirs)
# ycm_default_dirs(${VARS_PREFIX})
# #end superbuild stuff


set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(GYM REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(locoman_utils REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(tf_conversions REQUIRED)
FIND_PACKAGE(trajectory_generator REQUIRED)
FIND_PACKAGE(wholebody_ik REQUIRED)

file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

include_directories(include include/multicontact/ include/utils/)

# add include directories
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} 
                            ${YARP_INCLUDE_DIRS} 
                            ${iDynTree_INCLUDE_DIRS} 
                            ${idynutils_INCLUDE_DIRS}
                            ${GYM_INCLUDE_DIRS}
                            ${locoman_utils_INCLUDE_DIRS}
                            ${trajectory_generator}
                            ${wholebody_ik_INCLUDE_DIRS}
                            )

LINK_DIRECTORIES(${ROS_LIB_FOLDER} ${WALKMAN_LIB_FOLDER})

file(GLOB_RECURSE INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)


add_subdirectory(app)

ADD_EXECUTABLE(multicontact_module src/multicontact_module_main.cpp src/multicontact_module_thread.cpp src/utils/utils.cpp src/wb_interface/wb_interface.cpp)
                               
TARGET_LINK_LIBRARIES(multicontact_module ${wholebody_ik_LIBRARIES} ${iDynTree_LIBRARIES}
                                      ${YARP_LIBRARIES}
                                      ${idynutils_LIBRARIES}
                                      ${GYM_LIBRARIES}
                                      ${trajectory_generator_LIBRARIES}
                                      )

install(TARGETS multicontact_module DESTINATION bin)

