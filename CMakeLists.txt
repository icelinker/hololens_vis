cmake_minimum_required(VERSION 2.8.3)
project(hololens_vis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  tf
  image_transport
  cv_bridge
  roslib
  hololens_experiment
)

set(CMAKE_CXX_FLAGS  "-W -Wall -std=c++11")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hololens_vis
#  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(hololens_vis
#   src/${PROJECT_NAME}/hololens_vis.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(hololens_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(hololens_vis_node src/hololens_vis_node.cpp)
add_executable(holoLink src/hololink.cpp)
add_executable(imageToString src/imageToString.cpp)
add_executable(stringToImage src/stringToImage.cpp)
add_executable(timeSync src/timesync.cpp)
add_executable(exp_control src/exp_control.cpp)
add_executable(inv_trans src/inv_trans.cpp)
add_executable(holoLinkLight src/holoLinkLight.cpp)
add_executable(calibrateMarkersCam src/calibrateMarkersCam.cpp)
add_executable(calibrateCamInGun src/calibrateCamInGun.cpp)
add_executable(nozzleFilter src/nozzleFilter.cpp)
add_executable(markerPose src/markerPose.cpp)
add_executable(markerAgr src/markerAgr.cpp)
add_executable(findpath src/findpath.cpp src/stlastar.h src/fsa.h)
add_executable(spoofTF src/spoofTF.cpp)
add_executable(spoofImages src/spoofImages.cpp)
add_executable(distortForCam src/distortForCam.cpp)
add_executable(pointWorldAlignment src/pointWorldAlignment.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hololens_vis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pointWorldAlignment
${catkin_LIBRARIES}
)

target_link_libraries(holoLinkLight
${catkin_LIBRARIES}
)
target_link_libraries(holoLink
${catkin_LIBRARIES}
)
target_link_libraries(imageToString
    ${catkin_LIBRARIES}
)
target_link_libraries(distortForCam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
target_link_libraries(stringToImage
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
target_link_libraries(timeSync
    ${catkin_LIBRARIES}
)
target_link_libraries(exp_control
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

target_link_libraries(inv_trans
   ${catkin_LIBRARIES}
)
target_link_libraries(calibrateMarkersCam
   ${catkin_LIBRARIES}
)


target_link_libraries(calibrateCamInGun
  ${catkin_LIBRARIES}
)

target_link_libraries(nozzleFilter
  ${catkin_LIBRARIES}
)
target_link_libraries(markerPose
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
   ${OpenCV_LIBS}
)
target_link_libraries(markerAgr
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
   ${OpenCV_LIBS}
)
target_link_libraries(findpath
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)
target_link_libraries(spoofTF
        ${catkin_LIBRARIES}
)
target_link_libraries(spoofImages
        ${catkin_LIBRARIES}
)
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
# install(TARGETS hololens_vis hololens_vis_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hololens_vis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
