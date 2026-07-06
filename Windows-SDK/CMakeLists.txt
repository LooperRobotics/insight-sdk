cmake_minimum_required(VERSION 3.20)
project(Insight9SDK)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 消除 MSVC 编码警告
if(MSVC)
    add_compile_options("/utf-8")
endif()

# ---------- FFmpeg 选择 ----------
option(USE_SYSTEM_FFMPEG "Use system FFmpeg libraries" OFF)

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

# ---------- HIDAPI 选择 ----------
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

# ---------- SDK 库 ----------
add_library(Insight9SDK STATIC Insight9SDK.cpp ExtensionUnitControl.cpp)
target_include_directories(Insight9SDK PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_libraries(Insight9SDK PRIVATE
    ${FFMPEG_LIBRARIES}
    ${HIDAPI_TARGET}
    ole32
    strmiids
    uuid
)

# ---------- 示例程序 ----------
add_executable(example example.cpp)
target_link_libraries(example Insight9SDK)

# ---------- 自动复制运行时 DLL ----------
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