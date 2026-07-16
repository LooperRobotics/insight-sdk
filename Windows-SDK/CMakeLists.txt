cmake_minimum_required(VERSION 3.20)
project(Insight9SDK)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options("/utf-8")
endif()

option(USE_SYSTEM_FFMPEG "Use system FFmpeg libraries" OFF)

# OpenCV 路径设置
set(OPENCV_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opencv")
set(OPENCV_INCLUDE_DIR "${OPENCV_PROJECT_DIR}/build/include")
set(OPENCV_LIB_DIR "${OPENCV_PROJECT_DIR}/build/x64/vc16/lib")
set(OPENCV_BIN_DIR "${OPENCV_PROJECT_DIR}/build/x64/vc16/bin")

if(NOT EXISTS "${OPENCV_INCLUDE_DIR}")
    message(WARNING "OpenCV include not found. Please run download_opencv.bat first.")
endif()

# 查找 OpenCV 库（先找到库，稍后创建 example 后再复制 DLL）
set(OPENCV_FOUND FALSE)
if(EXISTS "${OPENCV_LIB_DIR}")
    find_library(OPENCV_WORLD_LIB 
        NAMES 
            opencv_world4120
            opencv_world480
            opencv_world
        PATHS "${OPENCV_LIB_DIR}"
        NO_DEFAULT_PATH
    )
    
    if(OPENCV_WORLD_LIB)
        message(STATUS "Found OpenCV: ${OPENCV_WORLD_LIB}")
        add_library(opencv INTERFACE)
        target_include_directories(opencv INTERFACE "${OPENCV_INCLUDE_DIR}")
        target_link_libraries(opencv INTERFACE "${OPENCV_WORLD_LIB}")
        set(OPENCV_FOUND TRUE)
    else()
        message(WARNING "OpenCV library not found in ${OPENCV_LIB_DIR}")
        file(GLOB OPENCV_LIBS "${OPENCV_LIB_DIR}/*.lib")
        message(STATUS "Available libs: ${OPENCV_LIBS}")
        add_library(opencv INTERFACE)
    endif()
else()
    message(WARNING "OpenCV lib directory not found: ${OPENCV_LIB_DIR}")
    add_library(opencv INTERFACE)
endif()

if(USE_SYSTEM_FFMPEG)
    find_package(FFMPEG REQUIRED COMPONENTS avcodec avdevice avformat avutil swscale)
else()
    set(FFMPEG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg-z16" CACHE PATH "Prebuilt FFmpeg")
    set(FFMPEG_INCLUDE_DIRS "${FFMPEG_DIR}/include")
    set(FFMPEG_LIBRARIES
        "${FFMPEG_DIR}/lib/avcodec.lib"
        "${FFMPEG_DIR}/lib/avformat.lib"
        "${FFMPEG_DIR}/lib/avdevice.lib"
        "${FFMPEG_DIR}/lib/avutil.lib"
        "${FFMPEG_DIR}/lib/swscale.lib"
        "${FFMPEG_DIR}/lib/swresample.lib"
    )
endif()

option(USE_SYSTEM_HIDAPI "Use system HIDAPI (requires vcpkg or manual installation)" OFF)

if(USE_SYSTEM_HIDAPI)
    find_package(hidapi CONFIG REQUIRED)
    set(HIDAPI_TARGET hidapi::hidapi)
else()
    set(HIDAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hidapi-prebuilt" CACHE PATH "Prebuilt HIDAPI")
    set(HIDAPI_INCLUDE_DIR "${HIDAPI_DIR}/include")
    set(HIDAPI_LIBRARY "${HIDAPI_DIR}/lib/hidapi.lib")
    add_library(hidapi_prebuilt STATIC IMPORTED)
    set_target_properties(hidapi_prebuilt PROPERTIES
        IMPORTED_LOCATION "${HIDAPI_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${HIDAPI_INCLUDE_DIR}"
    )
    set(HIDAPI_TARGET hidapi_prebuilt)
endif()

add_library(Insight9SDK STATIC Insight9SDK.cpp ExtensionUnitControl.cpp)
target_include_directories(Insight9SDK PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(Insight9SDK PRIVATE
    ${FFMPEG_LIBRARIES}
    ${HIDAPI_TARGET}
    ole32
    strmiids
    uuid
)

add_executable(example example.cpp)
target_link_libraries(example Insight9SDK opencv)

# 复制 OpenCV DLL（在 example 创建之后）
if(OPENCV_FOUND AND EXISTS "${OPENCV_BIN_DIR}")
    file(GLOB OPENCV_DLLS "${OPENCV_BIN_DIR}/opencv_world*.dll")
    if(OPENCV_DLLS)
        add_custom_command(TARGET example POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${OPENCV_DLLS}
                $<TARGET_FILE_DIR:example>
            COMMENT "Copying OpenCV DLLs"
        )
        message(STATUS "OpenCV DLLs will be copied: ${OPENCV_DLLS}")
    else()
        message(WARNING "OpenCV DLLs not found in ${OPENCV_BIN_DIR}")
    endif()
endif()

if(NOT USE_SYSTEM_FFMPEG AND EXISTS "${FFMPEG_DIR}/bin")
    file(GLOB FFMPEG_DLLS 
        "${FFMPEG_DIR}/bin/avcodec-*.dll"
        "${FFMPEG_DIR}/bin/avformat-*.dll"
        "${FFMPEG_DIR}/bin/avdevice-*.dll"
        "${FFMPEG_DIR}/bin/avfilter-*.dll"
        "${FFMPEG_DIR}/bin/avutil-*.dll"
        "${FFMPEG_DIR}/bin/swscale-*.dll"
        "${FFMPEG_DIR}/bin/swresample-*.dll"
    )
    foreach(DLL_FILE ${FFMPEG_DLLS})
        add_custom_command(TARGET example POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${DLL_FILE}"
                $<TARGET_FILE_DIR:example>
        )
    endforeach()
endif()

if(NOT USE_SYSTEM_HIDAPI AND EXISTS "${HIDAPI_DIR}/bin/hidapi.dll")
    add_custom_command(TARGET example POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${HIDAPI_DIR}/bin/hidapi.dll"
            $<TARGET_FILE_DIR:example>
    )
endif()