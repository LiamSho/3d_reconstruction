cmake_minimum_required(VERSION 3.21)
project(3d_reconstruction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

enable_language(C)
enable_language(CXX)

option(BUILD_WITH_CUDA "Build CUDA powered components" OFF)
if (BUILD_WITH_CUDA)
    message("-- CUDA: ON")
    add_definitions(-DUSE_CUDA)
    set(BUILD_VGICP_CUDA ON CACHE BOOL "Build GPU-powered VGICP" FORCE)
else ()
    message("-- CUDA: OFF")
endif ()
set(BUILD_apps OFF CACHE BOOL "Build application programs" FORCE)

add_subdirectory(./include/fast_gicp)
include_directories(./include)

find_package(realsense2 CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (OpenMP_CXX_FOUND)
    add_definitions(-DUSE_OPENMP)
    link_libraries(OpenMP::OpenMP_CXX)
endif ()

link_libraries(
        ${realsense2_LIBRARY}
        ${PCL_LIBRARIES}
        spdlog::spdlog_header_only
        glfw
        fast_gicp
)

add_subdirectory(src)
