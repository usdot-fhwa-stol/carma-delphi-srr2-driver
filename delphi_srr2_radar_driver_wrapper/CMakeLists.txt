
# Copyright (C) 2019-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 2.8.3)
project(delphi_srr2_radar_driver_wrapper)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cav_driver_utils
  cav_msgs
  delphi_srr_msgs
  radar_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cav_driver_utils cav_msgs delphi_srr_msgs radar_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
file(GLOB_RECURSE headers */*.hpp */*.h)
add_library(delphi_srr2_driver_wrapper_worker_library src/delphi_srr2_radar_driver_wrapper_worker.cpp)
add_dependencies(delphi_srr2_driver_wrapper_worker_library ${catkin_EXPORTED_TARGETS})
add_executable( ${PROJECT_NAME}_node
        ${headers}
        src/main.cpp
        src/delphi_srr2_radar_driver_wrapper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME}_node ${catkin_LIBRARIES} delphi_srr2_driver_wrapper_worker_library)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_delphi_srr2_radar_driver_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}-test delphi_srr2_driver_wrapper_worker_library)
