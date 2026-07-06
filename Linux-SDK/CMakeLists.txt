cmake_minimum_required(VERSION 3.10)
project(Insight9_SDK VERSION 1.1.0 LANGUAGES C CXX)

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 查找依赖库
find_package(Threads REQUIRED)

# 添加编译选项
add_compile_options(-Wall -Wextra -fPIC)

# 关键：禁用 Fortify Source 安全检查（这些会触发 buffer overflow 误报）
add_compile_options(-U_FORTIFY_SOURCE)
add_compile_options(-D_FORTIFY_SOURCE=0)

# 抑制警告
add_compile_options(-Wno-missing-field-initializers)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-class-memaccess)
add_compile_options(-Wno-format-truncation)
add_compile_options(-Wno-type-limits)
add_compile_options(-Wno-array-bounds)

# 添加 Release 模式下的优化选项（可选）
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
else()
    add_compile_options(-O0 -g)
endif()

# ==================== 动态库 ====================
set(LIB_SOURCES
    Insight_9_receive.cpp
    UvcExtensionUnit.cpp
)

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
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    OUTPUT_NAME "insight9"
)

# ==================== 示例程序 ====================
add_executable(example example.cpp)

target_link_libraries(example PRIVATE
    insight9
    Threads::Threads
)

# 设置 RPATH
set_target_properties(example PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
    INSTALL_RPATH "\$ORIGIN"
)

# ==================== 安装 ====================
install(TARGETS insight9
    LIBRARY DESTINATION lib
)

install(FILES ${LIB_HEADERS} DESTINATION include)
install(TARGETS example DESTINATION bin)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Fortify Source: disabled")