cmake_minimum_required(VERSION 3.16)
project(agv_description)
# Keep explicit Debug/multi-config builds available; normal simulation needs optimization.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config meshes DESTINATION share/${PROJECT_NAME})
ament_package()
