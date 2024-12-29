cmake_minimum_required(VERSION 3.10.0)
project(CircularHarmonics VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif()

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL2 CONFIG REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCLHeaders REQUIRED)
find_package(OpenCLICDLoader REQUIRED)
find_package(OpenCLHeadersCpp REQUIRED)

include_directories("include")
include_directories(3rdparty/imgui)
#include_directories(3rdparty/stb)
file(GLOB SOURCES src/*.cpp)
file(GLOB IMGUI_SOURCES 3rdparty/imgui/*.cpp)

add_library(Imgui ${IMGUI_SOURCES})
target_link_libraries(Imgui PRIVATE SDL2::SDL2-static SDL2::SDL2main)

add_library(CircHarm STATIC ${SOURCES})
target_link_libraries(CircHarm PRIVATE OpenCL::Headers OpenCL::OpenCL OpenCL::HeadersCpp)
target_compile_definitions(CircHarm PRIVATE CL_HPP_TARGET_OPENCL_VERSION=300)

if (WIN32)
    message(STATUS "host:win32")
    if ("${CMAKE_MAKE_PROGRAM}" MATCHES "MsBuild")
        add_custom_command(TARGET CircHarm
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/$(ConfigurationName)/assets
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/kernel ${CMAKE_BINARY_DIR}/$(ConfigurationName)/kernel
                )
    elseif(MINGW)
        add_custom_command(TARGET CircHarm
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/assets
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/kernel ${CMAKE_BINARY_DIR}/kernel
                )
    endif ()
endif ()

install(TARGETS CircHarm RUNTIME DESTINATION bin)

add_executable(CircHarmExample example/circharm_example.cpp)
target_link_libraries(CircHarmExample PRIVATE Imgui)
target_link_libraries(CircHarmExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(CircHarmExample PRIVATE CircHarm)

add_executable(CircHarmTrainExample example/circharm_train_example.cpp)
target_link_libraries(CircHarmTrainExample PRIVATE Imgui)
target_link_libraries(CircHarmTrainExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(CircHarmTrainExample PRIVATE CircHarm)

add_executable(CircHarmNormExample example/circharm_norm_example.cpp)
target_link_libraries(CircHarmNormExample PRIVATE Imgui)
target_link_libraries(CircHarmNormExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(CircHarmNormExample PRIVATE CircHarm)

add_executable(CircHarmAdvancedExample example/circharm_advanced_example.cpp)
target_link_libraries(CircHarmAdvancedExample PRIVATE Imgui)
target_link_libraries(CircHarmAdvancedExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(CircHarmAdvancedExample PRIVATE CircHarm)

add_executable(CircHarmAdvancedCollisionExample example/circharm_advanced_collision_example.cpp)
target_link_libraries(CircHarmAdvancedCollisionExample PRIVATE Imgui)
target_link_libraries(CircHarmAdvancedCollisionExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(CircHarmAdvancedCollisionExample PRIVATE CircHarm)

add_executable(CircHarmTest test/circharm_test.cpp)
target_link_libraries(CircHarmTest PRIVATE ${GTEST_BOTH_LIBRARIES} pthread)
target_link_libraries(CircHarmTest PRIVATE CircHarm)

gtest_discover_tests(CircHarmTest)