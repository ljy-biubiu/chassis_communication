# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cti_chassis_handdle_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cti_chassis_handdle_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cti_chassis_handdle_FOUND FALSE)
  elseif(NOT cti_chassis_handdle_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cti_chassis_handdle_FOUND FALSE)
  endif()
  return()
endif()
set(_cti_chassis_handdle_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cti_chassis_handdle_FIND_QUIETLY)
  message(STATUS "Found cti_chassis_handdle: 20.0.0 (${cti_chassis_handdle_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cti_chassis_handdle' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cti_chassis_handdle_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cti_chassis_handdle_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cti_chassis_handdle_DIR}/${_extra}")
endforeach()
