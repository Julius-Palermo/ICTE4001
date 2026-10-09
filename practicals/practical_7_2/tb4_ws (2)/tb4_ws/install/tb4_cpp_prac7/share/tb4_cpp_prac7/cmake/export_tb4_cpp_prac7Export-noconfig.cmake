#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tb4_cpp_prac7::iar_astar_planner" for configuration ""
set_property(TARGET tb4_cpp_prac7::iar_astar_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(tb4_cpp_prac7::iar_astar_planner PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libiar_astar_planner.so"
  IMPORTED_SONAME_NOCONFIG "libiar_astar_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tb4_cpp_prac7::iar_astar_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_tb4_cpp_prac7::iar_astar_planner "${_IMPORT_PREFIX}/lib/libiar_astar_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
