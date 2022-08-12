cmake_minimum_required(VERSION 3.5)

project(libmultirobotplanning)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(ExternalProject)

set(EXTERNAL_PROJECT_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-parameter")

ExternalProject_Add(libmultirobotplanning
  CMAKE_ARGS 
    -DCMAKE_CXX_FLAGS=${EXTERNAL_PROJECT_CMAKE_CXX_FLAGS}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/libMultiRobotPlanning
  BINARY_DIR ${PROJECT_BINARY_DIR}/install_cache
  INSTALL_COMMAND ""
)


##################
#  INSTALLATION  #
##################

## install include directory
install(DIRECTORY "external/libMultiRobotPlanning/include/" DESTINATION "include")

## install the planners
install(
  FILES 
    ${PROJECT_BINARY_DIR}/install_cache/assignment
    ${PROJECT_BINARY_DIR}/install_cache/a_star
    ${PROJECT_BINARY_DIR}/install_cache/a_star
    ${PROJECT_BINARY_DIR}/install_cache/cbs
    ${PROJECT_BINARY_DIR}/install_cache/cbs_roadmap
    ${PROJECT_BINARY_DIR}/install_cache/cbs_ta
    ${PROJECT_BINARY_DIR}/install_cache/ecbs
    ${PROJECT_BINARY_DIR}/install_cache/ecbs_ta
    ${PROJECT_BINARY_DIR}/install_cache/mapf_prioritized_sipp
    ${PROJECT_BINARY_DIR}/install_cache/next_best_assignment
    ${PROJECT_BINARY_DIR}/install_cache/sipp
  DESTINATION "bin"
)

