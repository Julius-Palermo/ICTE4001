# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tb4_cpp_prac7_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tb4_cpp_prac7_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tb4_cpp_prac7_FOUND FALSE)
  elseif(NOT tb4_cpp_prac7_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tb4_cpp_prac7_FOUND FALSE)
  endif()
  return()
endif()
set(_tb4_cpp_prac7_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tb4_cpp_prac7_FIND_QUIETLY)
  message(STATUS "Found tb4_cpp_prac7: 0.0.0 (${tb4_cpp_prac7_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tb4_cpp_prac7' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tb4_cpp_prac7_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tb4_cpp_prac7_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${tb4_cpp_prac7_DIR}/${_extra}")
endforeach()
