cmake_minimum_required(VERSION 2.8.3)

set(RGBD_MSG "/camera/depth_registered/points")
set(IMAGE_PATH "${IMAGE_OUTPUT_DIR}/colorImage.jpg")
set(DEPTH_IMAGE_PATH "${IMAGE_OUTPUT_DIR}/depthImage.jpg")

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/image_capture/config.h.in"
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/image_capture/config.h"
)

add_executable(image_capture
    main.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
)

target_link_libraries(image_capture
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
