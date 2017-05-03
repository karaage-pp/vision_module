cmake_minimum_required(VERSION 2.8.3)

set(TEMP_DIR "TEMP")
set(DATA_DIR "datas")
set(PICT_DIR "picts")
set(LEARNED_DIR "learned")
set(PY_INTERFACE "interface_face")
set(FEATURE_FILE "${FACEDATA_DIR}/FEAT:000:0000.txt")
set(AGE_LIST "${FACEDATA_DIR}/age_list.txt")
set(GENDER_LIST "${FACEDATA_DIR}/gender_list.txt")
set(FACE_MODEL "${FACEMODEL_DIR}/face_net.tf")

configure_file(
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/face_recognition/config.h.in" 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/face_recognition/config.h"
)

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

include_directories(
    ${PYTHON_INCLUDE_PATH}
)

add_executable(face_recognition
    main.cc
    FaceNet.cc
    ${PROJECT_SOURCE_DIR}/common/MySys.cc
    ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
    ${PROJECT_SOURCE_DIR}/common/svm.cc
    ${PROJECT_SOURCE_DIR}/common/TrainSVM.cc
)

target_link_libraries(face_recognition
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    python2.7
)

