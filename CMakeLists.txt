cmake_minimum_required(VERSION 3.0.2)
project(caric_competition_xmu)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
  octomap_ros
  message_generation
  pcl_conversions
  pcl_ros
)
#Eigen
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  )



add_message_files(
  FILES
  GridMap.msg
  GridNode.msg
  # 列出您的所有消息文件
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


add_definitions("-Wall -g")

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS},
  ${EIGEN3_INCLUDE_DIR},
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(ptcloud_2_octomap 
    src/ptcloud_2_octomap.cpp
)
add_executable(get_occupancy
    src/get_occupancy.cpp
)
add_executable(inspector
    src/inspector.cpp
)

add_executable(photo_taker
    src/photo_taker.cpp
) 

add_executable(k_means_test
    src/k_means_test.cpp
)

add_executable(a_star_test
    src/a_star_test.cpp)

add_executable(arrange_point_test
    src/arrange_test.cpp
    )
  
add_executable(subscriber_test
    src/subscriber.cpp
)

add_executable(controller_test
    src/controller.cpp
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
target_link_libraries(photo_taker
  ${catkin_LIBRARIES}
)
# 链接opencv库
target_link_libraries(k_means_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(a_star_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

target_link_libraries(arrange_point_test
  ${catkin_LIBRARIES}
)

target_link_libraries(subscriber_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(controller_test
  ${catkin_LIBRARIES}
)
#设置可执行文件的输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
