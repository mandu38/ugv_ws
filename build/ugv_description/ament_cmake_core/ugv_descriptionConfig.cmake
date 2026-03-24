# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ugv_description_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ugv_description_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ugv_description_FOUND FALSE)
  elseif(NOT ugv_description_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ugv_description_FOUND FALSE)
  endif()
  return()
endif()
set(_ugv_description_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ugv_description_FIND_QUIETLY)
  message(STATUS "Found ugv_description: 1.0.0 (${ugv_description_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ugv_description' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ugv_description_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ugv_description_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${ugv_description_DIR}/${_extra}")
endforeach()
