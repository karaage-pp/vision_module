cmake_minimum_required(VERSION 2.8.3)

set(PLANE_IMAGE_PATH "${IMAGE_OUTPUT_DIR}/plane.jpg")
set(FILTERED_IMAGE "${IMAGE_OUTPUT_DIR}/filtered.jpg")

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/plane_detection/config.h.in" 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/plane_detection/config.h"
)

add_executable(plane_detection
    main.cc
    PlaneDetection.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
    ${PROJECT_SOURCE_DIR}/common/VectorCalculus.cc
    ${PROJECT_SOURCE_DIR}/common/Segmentation.cc
)

target_link_libraries(plane_detection
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

