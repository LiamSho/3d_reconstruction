add_subdirectory(utils)

aux_source_directory(. DIR_MAIN_SRC)
add_library(main ${DIR_MAIN_SRC})
