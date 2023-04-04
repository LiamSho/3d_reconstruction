cmake_minimum_required(VERSION 3.21)
project(3d_reconstruction)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

enable_language(C)
enable_language(CXX)

message("-- Compiler: ${CMAKE_CXX_COMPILER_ID}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-ignored-optimization-argument)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-ignored-optimization-argument)
elseif (MSVC)
    add_compile_options(/W4 /WX)
endif ()

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
)

add_subdirectory(src)
