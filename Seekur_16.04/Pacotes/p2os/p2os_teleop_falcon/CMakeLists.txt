cmake_minimum_required(VERSION 2.8.3)
project(p2os_teleop_falcon)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs sensor_msgs tf)

catkin_package(
    CATKIN_DEPENDS geometry_msgs sensor_msgs tf std_msgs
    LIBRARIES lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(p2os_teleop_falcon src/p2os_teleop_falcon.cc)
target_link_libraries(p2os_teleop_falcon ${catkin_LIBRARIES})

add_executable(p2os_teleop_falcon2 src/p2os_teleop_falcon2.cc)
target_link_libraries(p2os_teleop_falcon2 ${catkin_LIBRARIES})

install(TARGETS p2os_teleop_falcon p2os_teleop_falcon2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
