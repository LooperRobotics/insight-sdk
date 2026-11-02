# Insight SDK PoE examples. In the SDK package (poe/examples) they build
# against the SDK next to them; from the deb (/usr/share/insightsdk/poe/examples)
# against /usr:
#
#   cmake -S examples -B build -DCMAKE_PREFIX_PATH=<poe folder> && cmake --build build
cmake_minimum_required(VERSION 3.10)
if(NOT TARGET LooperSDK::looper)
  project(looper_examples C CXX)
  set(CMAKE_CXX_STANDARD 11)
  find_package(LooperSDK 1.0 REQUIRED)
endif()

add_subdirectory(0.basic.enumerate)
add_subdirectory(1.device.forceip)
add_subdirectory(2.dds.profile)
add_subdirectory(3.stream.imu_pose)
add_subdirectory(4.stream.frameset)
add_subdirectory(5.stream.viewer)
