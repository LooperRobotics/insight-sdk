#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LooperSDK::looper" for configuration "Release"
set_property(TARGET LooperSDK::looper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LooperSDK::looper PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblooper.so.1.0.0"
  IMPORTED_SONAME_RELEASE "liblooper.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS LooperSDK::looper )
list(APPEND _IMPORT_CHECK_FILES_FOR_LooperSDK::looper "${_IMPORT_PREFIX}/lib/liblooper.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
