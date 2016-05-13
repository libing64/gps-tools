cmake_minimum_required(VERSION 2.8.3)
project(height_fusion)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

# generate messages
# add_message_files(FILES Height.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_definitions("-Wall -Werror -std=c++11")
add_executable(${PROJECT_NAME}
  src/height_fusion_node.cpp
  src/height_fusion.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
