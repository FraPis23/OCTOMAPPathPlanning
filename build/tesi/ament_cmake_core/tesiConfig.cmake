# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tesi_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tesi_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tesi_FOUND FALSE)
  elseif(NOT tesi_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tesi_FOUND FALSE)
  endif()
  return()
endif()
set(_tesi_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tesi_FIND_QUIETLY)
  message(STATUS "Found tesi: 0.0.0 (${tesi_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tesi' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tesi_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tesi_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tesi_DIR}/${_extra}")
endforeach()
