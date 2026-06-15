cmake_minimum_required(VERSION 3.10)

project(insight9_sdk LANGUAGES CXX)

find_package(Threads REQUIRED)

add_library(insight9 SHARED
    Insight_9_receive.cpp
    UvcExtensionUnit.cpp
)

target_include_directories(insight9
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(insight9 PUBLIC cxx_std_17)

target_link_libraries(insight9
    PUBLIC
        Threads::Threads
)

set_target_properties(insight9 PROPERTIES
    OUTPUT_NAME insight9
)

install(TARGETS insight9
    LIBRARY DESTINATION lib
)

install(FILES Insight_9_receive.h
    DESTINATION include
)
