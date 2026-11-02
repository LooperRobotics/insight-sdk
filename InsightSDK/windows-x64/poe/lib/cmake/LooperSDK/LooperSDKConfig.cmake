# LooperSDK CMake package.
#
#   find_package(LooperSDK 1.0 REQUIRED)
#   target_link_libraries(app PRIVATE LooperSDK::looper)
#
# The packages carry the shared library only. LooperSDK::looper_static, the
# static target of 1.0.0, is kept as another name for it, so projects that
# link it still build (and then need the shared library at run time).
#
# On Windows, copy looper.dll (bin/) next to the application.

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was LooperSDKConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)
find_dependency(Threads)

include(${CMAKE_CURRENT_LIST_DIR}/LooperSDKTargets.cmake)

if(TARGET LooperSDK::looper AND NOT TARGET LooperSDK::looper_static)
  add_library(LooperSDK::looper_static INTERFACE IMPORTED)
  set_property(TARGET LooperSDK::looper_static PROPERTY INTERFACE_LINK_LIBRARIES LooperSDK::looper)
endif()

# The Windows package is built with MinGW-w64. MSVC links the DLL through
# looper.lib, an import library made for it.
if(MSVC AND TARGET LooperSDK::looper)
  get_filename_component(_lp_prefix ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
  if(EXISTS ${_lp_prefix}/lib/looper.lib)
    foreach(_cfg "" _RELEASE _DEBUG _RELWITHDEBINFO _MINSIZEREL)
      set_property(TARGET LooperSDK::looper PROPERTY IMPORTED_IMPLIB${_cfg} ${_lp_prefix}/lib/looper.lib)
    endforeach()
  endif()
endif()

check_required_components(LooperSDK)
