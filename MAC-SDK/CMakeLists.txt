cmake_minimum_required(VERSION 3.16)
project(Insight9SDK_macOS VERSION 1.0.0 LANGUAGES C CXX)

if(NOT APPLE)
    message(FATAL_ERROR "MAC-SDK targets macOS. Use Linux-SDK or Windows-SDK on other platforms.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build libinsight9 as a shared library (.dylib)" OFF)
option(INSIGHT9_DEBUG_LOG "Print SDK [INFO]/[DEBUG] logs" OFF)
option(INSIGHT9_BUILD_EXAMPLE "Build the OpenCV example" ON)

# libusb >= 1.0.25 is needed to detach the macOS camera driver (Homebrew: brew install libusb)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0>=1.0.25)
find_package(Threads REQUIRED)

add_library(Insight9SDK
    Insight9SDK.cpp
    UvcDevice.cpp
    ExtensionUnitControl.cpp
)
target_include_directories(Insight9SDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Insight9SDK PRIVATE PkgConfig::LIBUSB Threads::Threads)
target_compile_options(Insight9SDK PRIVATE -Wall -Wextra -Wno-unused-parameter)
if(INSIGHT9_DEBUG_LOG)
    target_compile_definitions(Insight9SDK PRIVATE DEBUG)
endif()
set_target_properties(Insight9SDK PROPERTIES OUTPUT_NAME insight9)

if(INSIGHT9_BUILD_EXAMPLE)
    # No COMPONENTS list: OpenCV_LIBS then holds every module, so the example
    # links whichever module provides a function (OpenCV 5 moved some, e.g.
    # colour conversion, out of the 4.x modules).
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        add_executable(example example.cpp)
        # Pass OpenCV's headers as -I placed first, not -isystem: otherwise another
        # OpenCV reachable through CPATH or /usr/local wins, and its headers no
        # longer match the library being linked (undefined cv:: symbols).
        set_target_properties(example PROPERTIES NO_SYSTEM_FROM_IMPORTED ON)
        target_include_directories(example BEFORE PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(example PRIVATE Insight9SDK ${OpenCV_LIBS} Threads::Threads)
        message(STATUS "OpenCV ${OpenCV_VERSION} found, building example")
        message(STATUS "OpenCV config  : ${OpenCV_DIR}")
        message(STATUS "OpenCV headers : ${OpenCV_INCLUDE_DIRS}")
    else()
        message(WARNING "OpenCV not found (brew install opencv); the example will not be built.")
    endif()
endif()

install(TARGETS Insight9SDK
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(FILES Insight9SDK.h DESTINATION include)
if(TARGET example)
    install(TARGETS example DESTINATION bin)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "libusb: ${LIBUSB_VERSION}")
