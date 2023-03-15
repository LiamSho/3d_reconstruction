set(TDR_LIBS reconstructor)

add_library(tdr INTERFACE)

add_subdirectory(${TDR_LIBS})

target_link_libraries(tdr INTERFACE ${TDR_LIBS})
