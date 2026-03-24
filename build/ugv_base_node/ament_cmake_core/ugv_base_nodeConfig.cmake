# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ugv_base_node_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ugv_base_node_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ugv_base_node_FOUND FALSE)
  elseif(NOT ugv_base_node_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ugv_base_node_FOUND FALSE)
  endif()
  return()
endif()
set(_ugv_base_node_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ugv_base_node_FIND_QUIETLY)
  message(STATUS "Found ugv_base_node: 0.0.0 (${ugv_base_node_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ugv_base_node' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ugv_base_node_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ugv_base_node_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ugv_base_node_DIR}/${_extra}")
endforeach()
