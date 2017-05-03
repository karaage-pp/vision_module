cmake_minimum_required(VERSION 2.8.3)

set(IMAGE_PATH "${IMAGE_OUTPUT_DIR}/objRecogImg.jpg")
set(MODEL "${MODEL_DIR}/bvlc_reference_caffenet.tf")
set(LABEL "${MODEL_DIR}/imagenet-classes.txt")
set(SVM_MODEL "${MODEL_DIR}/model.libsvm")

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/object_recognition/config.h.in" 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/object_recognition/config.h"
)

include_directories(${PYTHON_INCLUDE_DIR})

add_executable(object_recognition
    main.cc
    Alexnet.cc
    ImageProc.cc
    ${PROJECT_SOURCE_DIR}/common/MySys.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
    ${PROJECT_SOURCE_DIR}/common/svm.cc
    ${PROJECT_SOURCE_DIR}/common/TrainSVM.cc
)

target_link_libraries(object_recognition
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    python2.7
)

