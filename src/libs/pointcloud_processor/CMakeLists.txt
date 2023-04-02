set(LIB_NAME pointcloud_processor)

add_library(${LIB_NAME})

file(GLOB SRC_CPP *.cpp)
file(GLOB SRC_HPP *.hpp)

message("-- Add lib: ${LIB_NAME} ")
message("--   SRC_CPP: ${SRC_CPP}")
message("--   SRC_HPP: ${SRC_HPP}")

target_sources(${LIB_NAME} PRIVATE ${SRC_CPP} PUBLIC ${SRC_HPP})

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
