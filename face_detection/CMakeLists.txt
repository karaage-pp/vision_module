cmake_minimum_required(VERSION 2.8.3)

set(IMAGE_PATH "${PROJECT_SOURCE_DIR}/faceImage.jpg")

if(EXISTS "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
    message("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
    set(MODEL "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
elseif(EXISTS "/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
    message("/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
    set(MODEL "/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml")
else()
    message(FATAL_ERROR "No such file haarcascades_frontalface_alt.xml")
endif()

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/face_detection/config.h.in" 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/face_detection/config.h"
)

add_executable(face_detection
    main.cc
    FaceDetection.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
)

target_link_libraries(face_detection
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

