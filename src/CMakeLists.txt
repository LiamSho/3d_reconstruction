add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(utils)

target_link_libraries(${PROJECT_NAME} INTERFACE utils)
