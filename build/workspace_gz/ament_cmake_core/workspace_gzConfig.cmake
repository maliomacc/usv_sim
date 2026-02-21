# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_workspace_gz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED workspace_gz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(workspace_gz_FOUND FALSE)
  elseif(NOT workspace_gz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(workspace_gz_FOUND FALSE)
  endif()
  return()
endif()
set(_workspace_gz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT workspace_gz_FIND_QUIETLY)
  message(STATUS "Found workspace_gz: 0.0.1 (${workspace_gz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'workspace_gz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${workspace_gz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(workspace_gz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${workspace_gz_DIR}/${_extra}")
endforeach()
