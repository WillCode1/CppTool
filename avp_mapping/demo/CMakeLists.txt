set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

if(ENABLE_VIEWER)
    list(APPEND DEPEND_LIBS  ${Pangolin_LIBRARIES} pthread )
endif()

add_executable( test_hpa_mapping
     test_hpa_mapping.cc
     dataloader.cc
    )
target_link_libraries( test_hpa_mapping PRIVATE
   ${DEPEND_LIBS} ${PROJECT_NAME}
)


add_executable(nm_feature_mapping_demo
    nm_feature_mapping.cc
    dataloader.cc)
target_link_libraries(nm_feature_mapping_demo
    ${DEPEND_LIBS} ${PROJECT_NAME})

add_executable(test_remap
    test_remap.cc )
target_link_libraries( test_remap  ${DEPEND_LIBS}  ${PROJECT_NAME})

add_executable(transform_map
   transform_map.cc )
target_link_libraries( transform_map  ${DEPEND_LIBS}  ${PROJECT_NAME})


include_directories( ${PROJECT_SOURCE_DIR}/demo)

if(ENABLE_VIEWER)
    add_executable(localization
        localization_demo.cc
        multicamtracking.cc
        mapviewer/mapviewer.cc)
    target_link_libraries(localization
        ${DEPEND_LIBS}
        ${PROJECT_NAME} )
endif()
