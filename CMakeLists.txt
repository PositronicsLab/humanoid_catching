cmake_minimum_required(VERSION 2.8.3)
project(humanoid_catching)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/humanoid_catching/CMakeModules/")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    geometry_msgs
    message_filters
    actionlib
    actionlib_msgs
    tf
    sensor_msgs
    orocos_kdl
    pluginlib
    resource_retriever
    operational_space_controllers_msgs
    kinematics_cache
    rosbag
    shape_msgs
)

if (CATKIN_ENABLE_TESTING)
find_package(rostest REQUIRED)
endif (CATKIN_ENABLE_TESTING)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Ravelin REQUIRED)
find_package(Moby REQUIRED)
find_package(QPOASES REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(BLAS REQUIRED)
find_package(Gperftools)

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
add_message_files(
   FILES
   Contact.msg
   FallPoint.msg
   Limit.msg
   Shape.msg
   Link.msg
   IKMetric.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PredictFall.srv
   CalculateTorques.srv
   CreateMeshCache.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs actionlib_msgs shape_msgs
)

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
  DEPENDS system_lib
)

###########
## Build ##
###########

find_library(
   ODE_LIBRARY
   NAMES libode ode
   PATHS /usr/lib /usr/local/lib
)
add_definitions(-DdDOUBLE)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
    add_definitions(-DCATKIN_ENABLE_TESTING)
endif(CATKIN_ENABLE_TESTING)

add_executable(catch_human_controller src/catch_human_controller.cpp)
add_dependencies(catch_human_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(catch_human_controller
   ${catkin_LIBRARIES} ${RAVELIN_LIBRARIES} ${LIBXML2_LIBRARIES} kinematics_cache
)

add_executable(fall_predictor src/fall_predictor.cpp)
add_dependencies(fall_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fall_predictor
   ${catkin_LIBRARIES} ${ODE_LIBRARY}
)

add_executable(imu_visualizer src/imu_visualizer.cpp)
add_dependencies(imu_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_visualizer
   ${catkin_LIBRARIES}
)

add_executable(fall_starter src/fall_starter.cpp)
add_dependencies(fall_starter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fall_starter
   ${catkin_LIBRARIES}
)

add_executable(fall_readiness src/fall_readiness.cpp)
add_dependencies(fall_readiness ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fall_readiness
   ${catkin_LIBRARIES}
)

add_executable(balancer src/balancer.cpp)
add_dependencies(balancer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(balancer
   ${catkin_LIBRARIES} ${RAVELIN_LIBRARIES} ${BLAS_LIBRARIES} ${MOBY_LIBRARIES} ${QPOASES_LIBRARIES}
)

add_executable(calc_ee_bb src/calc_ee_bb.cpp)
add_dependencies(calc_ee_bb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calc_ee_bb
   ${catkin_LIBRARIES}
)

add_executable(fall_stats_recorder src/fall_stats_recorder.cpp)
add_dependencies(fall_stats_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fall_stats_recorder
   ${catkin_LIBRARIES}
)

add_executable(ros_bag_recorder src/ros_bag_recorder.cpp)
add_dependencies(ros_bag_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_bag_recorder
   ${catkin_LIBRARIES}
)

add_executable(on_ground_detector src/on_ground_detector.cpp)
add_dependencies(on_ground_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(on_ground_detector
   ${catkin_LIBRARIES}
)

add_executable(imu_filter src/imu_filter.cpp)
add_dependencies(imu_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_filter
   ${catkin_LIBRARIES}
)

add_executable(imu_gaussian_noise src/imu_gaussian_noise.cpp)
add_dependencies(imu_gaussian_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_gaussian_noise
   ${catkin_LIBRARIES}
)

add_executable(metrics_recorder src/metrics_recorder.cpp)
add_dependencies(metrics_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(metrics_recorder
   ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_catch_human_controller test/test_catch_human_controller.cpp src/catch_human_controller.cpp)
    set_target_properties(test_catch_human_controller PROPERTIES COMPILE_FLAGS -DENABLE_TESTING)
    target_link_libraries(test_catch_human_controller
        ${catkin_LIBRARIES} ${RAVELIN_LIBRARIES} ${LIBXML2_LIBRARIES} kinematics_cache
    )
    add_rostest_gtest(test_fall_predictor test/test_fall_predictor.test test/test_fall_predictor.cpp)
    target_link_libraries(test_fall_predictor ${catkin_LIBRARIES})

endif(CATKIN_ENABLE_TESTING)
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_humanoid_catching.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
