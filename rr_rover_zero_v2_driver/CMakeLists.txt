cmake_minimum_required(VERSION 2.8.3)
project(rr_rover_zero_v2_driver)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  std_msgs
  rr_rover_zero_v2_driver_msgs
  serial
  # message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf
  diagnostic_msgs
  urdf
  xacro
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pluginlib roscpp std_msgs rr_rover_zero_v2_driver_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(rr_rover_zero_v2_driver_node src/rr_rover_zero_v2_driver_node.cpp
                                src/rr_rover_zero_v2_driver.cpp
                                src/vesc_interface.cpp
                                src/vesc_packet.cpp
                                src/vesc_packet_factory.cpp)
add_dependencies(rr_rover_zero_v2_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rr_rover_zero_v2_driver_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rr_rover_zero_v2_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO
