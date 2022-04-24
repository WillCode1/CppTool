file(GLOB SRC_FILES
    "avp_mapping/*.cc"
    "avp_mapping/*.cu"
    "avp_mapping/g2o/*cc"
    "message_filter/*.cc"
)


file(GLOB FEATURE_MAPPING_SRC
    "feature_mapping/*.cc"
    "feature_mapping/g2o/*.cc"
    "feature_mapping/optimizer/*.cc"
    "message_filter/*.cc"
    )

file(GLOB FEATURE_VIEWER_FILES "feature_mapping/viewer/*.cc")

if(ENABLE_VIEWER)
    list(APPEND DEPEND_LIBS  ${Pangolin_LIBRARIES} pthread )
    list(APPEND SRC_FILES viewer/hpa_viewer.cc viewer/plot_viewer.cc)
    list(APPEND FEATURE_MAPPING_SRC  ${FEATURE_VIEWER_FILES})
endif()




if(NOT TDA4)
    file(COPY  "${PROJECT_SOURCE_DIR}/include/interface/avp_mapping_interface.h"
               "${PROJECT_SOURCE_DIR}/include/interface/vslam_types.h"
        DESTINATION  "${PROJECT_SOURCE_DIR}/lib/")
else()
    file(COPY  "${PROJECT_SOURCE_DIR}/include/interface/avp_mapping_interface.h"
               "${PROJECT_SOURCE_DIR}/include/interface/vslam_types.h"
        DESTINATION  "${PROJECT_SOURCE_DIR}/build/lib/")
endif()


add_library(${PROJECT_NAME} SHARED
     ${SRC_FILES}  ${FEATURE_MAPPING_SRC}
    )

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${DEPEND_LIBS} 
    )

#target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
