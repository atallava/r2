cmake_minimum_required(VERSION 2.8.3)
project(basil_ros_support)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

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
INCLUDE_DIRS include
CATKIN_DEPENDS std_msgs roscpp
LIBRARIES basil_ros_support
#DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include $ENV{VMI_MPC_INCLUDE_PATH})
include_directories(include $ENV{VMI_MPC_EXTERNALS_INCLUDE_PATH})
include_directories(include $ENV{VMI_MPC_EXTERNALS_PATH}/nrec/src)
include_directories(${catkin_INCLUDE_DIRS})

# link to the VMI's external library paths
link_directories($ENV{VMI_MPC_LIB_PATH})
link_directories($ENV{VMI_MPC_EXTERNALS_LIB_PATH}/boost)

#Declare a cpp library
add_library(basil_ros_support src/sinsmsg.cpp)

## Declare a cpp executable
#add_executable(basil_ros_support src/sinsmsg.cpp)

#
set(CMAKE_CXX_FLAGS "-DBOOST_PARAMETER_MAX_ARITY=7")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pathtracker_mpc_node pathtracker_mpc_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(basil_ros_support
   boost_serialization
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

# # Mark executables and/or libraries for installation
# install(TARGETS basil_ros_support
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Headers for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pathtracker_mpc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
