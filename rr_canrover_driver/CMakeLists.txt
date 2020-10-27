cmake_minimum_required(VERSION 2.8.3)
project(rr_canrover_driver)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  tf2
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rr_canrover_driver_node src/canrover_driver.cpp)
add_dependencies(rr_canrover_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rr_canrover_driver_node ${catkin_LIBRARIES}

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rr_canrover_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

