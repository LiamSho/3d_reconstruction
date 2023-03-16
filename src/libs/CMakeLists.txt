set(TDR_LIBS
        realsense_operator
        pointcloud_aligner)

add_library(tdr INTERFACE)

foreach (TDR_LIB ${TDR_LIBS})
    add_subdirectory(${TDR_LIB})
    target_link_libraries(tdr INTERFACE ${TDR_LIB})
endforeach ()
