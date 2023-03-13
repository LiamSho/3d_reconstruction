cmake_minimum_required(VERSION 3.25)
project(3d_reconstruction CXX)

set(CMAKE_CXX_STANDARD 20)

# Packages
find_package(realsense2 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

aux_source_directory(./src DIR_SRCS)

# Executable
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# Links
target_link_libraries(${PROJECT_NAME} PRIVATE ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${spdlog_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
