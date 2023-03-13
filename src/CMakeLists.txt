aux_source_directory(. DIR_LIB_SRCS)
message("common_path_cmake:" ${DIR_LIB_SRCS})
add_library(main ${DIR_LIB_SRCS})
