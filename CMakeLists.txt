cmake_minimum_required(VERSION 3.0.2)
project(caric_competition_xmu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

include_directories(
# include
  ${catkin_INCLUDE_DIRS},
  ${EIGEN3_INCLUDE_DIR},
  ${PROJECT_SOURCE_DIR}/include
)



find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
  octomap_ros
  nav_msgs
  message_generation
  pcl_conversions
  pcl_ros
  caric_mission

)
add_message_files(
  FILES
  GridNodeMsg.msg
  GridMapMsg.msg
  OdometryArray.msg
  SearchPointArray.msg
  SearchPointMsg.msg
  # 其他消息文件
)
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  caric_mission
  caric_competition_xmu
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 2023caric_competition_xmu
  CATKIN_DEPENDS mavros roscpp rospy std_msgs message_runtime nav_msgs caric_mission
#  DEPENDS system_lib
)

#Eigen
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/caric_mission
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS} # 如果有 PCL 依赖，请确保添加
  ${OpenCV_INCLUDE_DIR}
)

add_definitions("-Wall -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0  -Wall")




link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ptcloud_2_octomap 
    src/inspector/ptcloud_2_octomap.cpp
)
add_executable(get_occupancy
    src/inspector/get_occupancy.cpp
)
add_executable(inspector
    src/inspector/inspector.cpp
    src/inspector/trajectory_generator_node.cpp
    src/inspector/trajectory_generator.cpp
)

add_executable(subscriber_test
    src/photo_taker/subscriber_copy.cpp
    src/inspector/trajectory_generator_node.cpp
    src/inspector/trajectory_generator.cpp
)

add_executable(controller_test
    src/photo_taker/controller.cpp
)

add_executable(gcs
    src/photo_taker/gcs.cpp
)


target_link_libraries(ptcloud_2_octomap
  ${catkin_LIBRARIES}
)
target_link_libraries(get_occupancy
  ${catkin_LIBRARIES}
)
target_link_libraries(inspector
  ${catkin_LIBRARIES}
)

target_link_libraries(subscriber_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(controller_test
  ${catkin_LIBRARIES}
)

target_link_libraries(gcs
  ${catkin_LIBRARIES}
  )

catkin_install_python(PROGRAMS src/inspector/get_points_in_faces.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/inspector/inspector1.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/inspector/monitor.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# add_executable(gcs
#   src/photo_taker/gcs.cpp
# )

# add_executable(photo_taker
#     src/photo_taker/photo_taker.cpp
# ) 

# target_link_libraries(photo_taker
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(gcs
#   ${catkin_LIBRARIES}
# )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)



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


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/2023caric_competition_xmu.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/2023caric_competition_xmu_node.cpp)


#add_executable(ptcloud_2_octomap             # 重复
#    src/photo_taker/ptcloud_2_octomap.cpp
#)
#add_executable(get_occupancy                 # 重复
#    src/photo_taker/get_occupancy.cpp
#)
# add_executable(inspector
#     src/photo_taker/inspector.cpp
# )



# add_executable(k_means_test
#     src/photo_taker/k_means_test.cpp
# )

# add_executable(a_star_test
#     src/photo_taker/a_star_test.cpp)

# add_executable(arrange_point_test
#     src/photo_taker/arrange_test.cpp
#     )
  







# add_executable(test_in_sight
#     src/test_in_sight.cpp
# )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


#target_link_libraries(ptcloud_2_octomap        # 重复
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(get_occupancy            # 重复
#  ${catkin_LIBRARIES}
#)
# target_link_libraries(inspector
#   ${catkin_LIBRARIES}
# )

# 链接opencv库
# target_link_libraries(k_means_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )

# target_link_libraries(a_star_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS})

# target_link_libraries(arrange_point_test
#   ${catkin_LIBRARIES}
# )




# target_link_libraries(test_in_sight
#     ${catkin_LIBRARIES}
# )

# target_link_libraries(communicator
#   ${catkin_LIBRARIES}
# )
#设置可执行文件的输出路径
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)		# Added
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_2023caric_competition_xmu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
