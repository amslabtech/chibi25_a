# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_team_local_goal_creator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED team_local_goal_creator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(team_local_goal_creator_FOUND FALSE)
  elseif(NOT team_local_goal_creator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(team_local_goal_creator_FOUND FALSE)
  endif()
  return()
endif()
set(_team_local_goal_creator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT team_local_goal_creator_FIND_QUIETLY)
  message(STATUS "Found team_local_goal_creator: 0.0.0 (${team_local_goal_creator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'team_local_goal_creator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${team_local_goal_creator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(team_local_goal_creator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${team_local_goal_creator_DIR}/${_extra}")
endforeach()
