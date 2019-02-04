cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_description)
find_package(xacro REQUIRED)

#include(${xacro_PACKAGE_PATH}/cmake/xacroConfig-version.cmake)

#include(${CATKIN_GLOBAL_SHARE_DESTINATION}/xacro/cmake/xacroConfig-version.cmake)
#include(/home/itv/catkin_ws/devel/share/xacro/cmake/xacroConfig-version.cmake)



find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

set(FLIR_PTU_DEVEL_SHARE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(MAKE_DIRECTORY ${FLIR_PTU_DEVEL_SHARE}/urdf)

xacro_add_xacro_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/example.urdf.xacro
  ${FLIR_PTU_DEVEL_SHARE}/urdf/example.urdf)
add_custom_target(flir_ptu_example_urdf ALL
  DEPENDS ${FLIR_PTU_DEVEL_SHARE}/urdf/example.urdf)

install(FILES ${FLIR_PTU_DEVEL_SHARE}/urdf/example.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
