cmake_minimum_required(VERSION 3.5)
project(libmultirobotplanning)

# Only build libMultiRobotPlanning if GIT submodule is cloned,
#   otherwise assume user does not wish to install this automatically.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libMultiRobotPlanning/CMakeLists.txt")


  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()

  include(ExternalProject)

  # The upstream libMultiRobotPlanning throws several warnings during build time
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
    DESTINATION bin
    PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ
  )

else(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libMultiRobotPlanning/CMakeLists.txt")
  # Dummy call to install to avoid colcon warning: "Could not run installation step for ..."
  install(FILES)
endif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libMultiRobotPlanning/CMakeLists.txt")
