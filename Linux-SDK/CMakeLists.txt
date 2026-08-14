cmake_minimum_required(VERSION 3.10)
project(Insight9_SDK VERSION 1.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

add_compile_options(-Wall -Wextra -fPIC)

add_compile_options(-U_FORTIFY_SOURCE)
add_compile_options(-D_FORTIFY_SOURCE=0)

add_compile_options(-Wno-missing-field-initializers)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-class-memaccess)
add_compile_options(-Wno-format-truncation)
add_compile_options(-Wno-type-limits)
add_compile_options(-Wno-array-bounds)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
else()
    add_compile_options(-O0 -g)
endif()

set(LIB_SOURCES
    Insight_9_receive.cpp
    hid.c
    UvcExtensionUnit.cpp
)

set_source_files_properties(hid.c PROPERTIES LANGUAGE CXX)

set(LIB_HEADERS
    Insight_9_receive.h
    UvcExtensionUnit.hpp
)

add_library(insight9 SHARED ${LIB_SOURCES})

target_include_directories(insight9 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(insight9 PRIVATE
    Threads::Threads
)

set_target_properties(insight9 PROPERTIES
    # VERSION ""
    # SOVERSION ""
    OUTPUT_NAME "insight9"
)

add_executable(example example.cpp)

target_link_libraries(example PRIVATE
    insight9
    Threads::Threads
    ${OpenCV_LIBS}
)

target_include_directories(example PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(example PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
    INSTALL_RPATH "\$ORIGIN"
)

install(TARGETS insight9
    LIBRARY DESTINATION lib
)

install(FILES ${LIB_HEADERS} DESTINATION include)
install(TARGETS example DESTINATION bin)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Fortify Source: disabled")