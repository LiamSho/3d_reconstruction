cmake_minimum_required(VERSION 3.25)
project(3d_reconstruction CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

message("-- Compiler: ${CMAKE_CXX_COMPILER_ID}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-ignored-optimization-argument)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-ignored-optimization-argument)
elseif (MSVC)
    add_compile_options(/W4 /WX)
endif ()

find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(spdlog REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIB_SPDLOG spdlog::spdlog_header_only)
set(LIB_PCL ${PCL_LIBRARIES})
set(LIB_GLFW glfw)
set(LIB_RS ${realsense2_LIBRARY})

link_libraries(${LIB_SPDLOG} ${LIB_PCL} ${LIB_GLFW} ${LIB_RS})

add_subdirectory(src)
