# Insight 9 (UVC) SDK example, built against the SDK:
#
#   cmake -S examples -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
#
# The SDK is found next to this folder (uvc/include, uvc/lib of the tar.gz or
# zip) or in the system (the deb, under /usr).
cmake_minimum_required(VERSION 3.10)
project(insight9_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

find_path(INSIGHT9_INCLUDE_DIR Insight_9_receive.h HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
find_library(INSIGHT9_LIBRARY insight9 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
if(NOT INSIGHT9_INCLUDE_DIR OR NOT INSIGHT9_LIBRARY)
  message(FATAL_ERROR "Insight 9 SDK not found (Insight_9_receive.h, libinsight9.so)")
endif()

add_executable(example example.cpp)
target_include_directories(example PRIVATE ${INSIGHT9_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(example PRIVATE ${INSIGHT9_LIBRARY} Threads::Threads ${OpenCV_LIBS})
get_filename_component(_insight9_libdir ${INSIGHT9_LIBRARY} DIRECTORY)
set_target_properties(example PROPERTIES BUILD_RPATH ${_insight9_libdir})
