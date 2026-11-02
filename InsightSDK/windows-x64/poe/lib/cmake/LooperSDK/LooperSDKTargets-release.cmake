#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LooperSDK::looper" for configuration "Release"
set_property(TARGET LooperSDK::looper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LooperSDK::looper PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/looper.dll.a"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/looper.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS LooperSDK::looper )
list(APPEND _IMPORT_CHECK_FILES_FOR_LooperSDK::looper "${_IMPORT_PREFIX}/lib/looper.dll.a" "${_IMPORT_PREFIX}/bin/looper.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
