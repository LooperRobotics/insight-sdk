# Needs OpenCV to decode and display; skipped without it.
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs highgui)
if(OpenCV_FOUND)
    add_executable(viewer viewer.cpp)
    target_link_libraries(viewer PRIVATE LooperSDK::looper ${OpenCV_LIBS})
else()
    message(STATUS "OpenCV not found: examples/5.stream.viewer is not built")
endif()
