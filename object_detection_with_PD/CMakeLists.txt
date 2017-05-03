cmake_minimum_required(VERSION 2.8.3)

set(OBJECT_IMAGE "${IMAGE_OUTPUT_DIR}/objectImage.jpg")
set(RAW_OBJECT_IMAGE "${IMAGE_OUTPUT_DIR}/rawObjectImage.jpg")

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/object_detection_with_PD/config.h.in" 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/object_detection_with_PD/config.h"
)

add_executable(object_detection_with_PD 
    main.cc
    ObjectDetection.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
    ${PROJECT_SOURCE_DIR}/common/VectorCalculus.cc
    ${PROJECT_SOURCE_DIR}/common/Segmentation.cc
)

target_link_libraries(object_detection_with_PD
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

