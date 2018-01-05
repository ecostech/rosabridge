cmake_minimum_required(VERSION 2.8.3)
project(rosabridge_server)

find_package(catkin REQUIRED COMPONENTS roscpp rosabridge_msgs std_msgs nav_msgs sensor_msgs topic_tools tf)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosabridge_msgs std_msgs nav_msgs sensor_msgs topic_tools tf
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_odom_proxy_node src/odom_proxy_node.cpp)
target_link_libraries(${PROJECT_NAME}_odom_proxy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_odom_proxy_node PROPERTIES OUTPUT_NAME odom_proxy_node PREFIX "")
add_dependencies(${PROJECT_NAME}_odom_proxy_node rosabridge_msgs_gencpp)

add_executable(${PROJECT_NAME}_imu_proxy_node src/imu_proxy_node.cpp)
target_link_libraries(${PROJECT_NAME}_imu_proxy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_imu_proxy_node PROPERTIES OUTPUT_NAME imu_proxy_node PREFIX "")
add_dependencies(${PROJECT_NAME}_imu_proxy_node rosabridge_msgs_gencpp)

install(
  TARGETS ${PROJECT_NAME}_odom_proxy_node ${PROJECT_NAME}_imu_proxy_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
