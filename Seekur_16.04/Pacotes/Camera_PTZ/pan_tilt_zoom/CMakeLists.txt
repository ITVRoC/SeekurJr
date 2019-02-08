cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_zoom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  pan_tilt_goal.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    pan_tilt_zoom
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pan_tilt_zoom
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(include)
include_directories(include/pan_tilt_zoom
  ${catkin_INCLUDE_DIRS} ${pan_tilt_zoom_INCLUDE_DIRS}
)

# Add aria path
find_package(Aria QUIET)
if(Aria_FOUND)
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/include/Aria.h")
	  add_definitions(-DADEPT_PKG)
	  include_directories( /usr/local/Aria/include)
	  link_directories(/usr/local/Aria/lib)
	endif()
endif()

add_library(ArCameraMode_lib
  "src/ArCameraMode.cpp"
)

add_executable(ptz_commander_node src/ptz_commander_node.cpp)

add_dependencies(ptz_commander_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# "Aria pthread dl rt" was added to resolve the error of 'undefined reference to Aria::init()'
target_link_libraries(ptz_commander_node
  ArCameraMode_lib ${catkin_LIBRARIES} Aria pthread dl rt
)