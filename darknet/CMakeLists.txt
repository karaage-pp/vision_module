cmake_minimum_required(VERSION 2.8.3)

# set relative path from vision_module WORKSPACE
set(OBJECT_IMAGE "image/darknetImage.jpg")
set(COCO_DATA "darknet_cfg/coco.data")
set(YOLO_CFG "darknet_cfg/yolo.cfg")
set(WEIGHT "darknet_cfg/yolo.weights")
set(NAME_LIST "darknet_cfg/names.list")
set(THRESH ".24")

set(DARKNET_COMPILE_FLAGS "-Wall -Wfatal-errors -fPIC -Wno-sign-compare")
set(DARKNET_COMPILE_FLAGS "${DARKNET_COMPILE_FLAGS} -Ofast -Wl,--export-dynamic")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in" 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/config.h"
)
configure_file(
    "cfg/coco.data.in" 
    "${PROJECT_SOURCE_DIR}/cfg/coco.data"
)


set(SRC_DIR src)

find_package(Threads)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED python)
find_package(PythonLibs REQUIRED)
find_package(CUDA)
list(APPEND LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

if(CUDA_FOUND)
    set(CUDA_NVCC_FLAGS
        ${CUDA_NVCC_FLAGS};
        -O3
    )
    list(APPEND CU_LIBRARIES 
            ${CUDA_LIBRARIES} 
            ${CUDA_CUBLAS_LIBRARIES} 
            ${CUDA_curand_LIBRARY}
            ${CUDA_cusparse_LIBRARY}
    )
    add_definitions(-DGPU)
    cuda_include_directories(src)
    FILE(GLOB KERNELS ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/*.cu)
    LIST(REMOVE_ITEM KERNELS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/yolo_kernels.cu")
    LIST(REMOVE_ITEM KERNELS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/deconvolutional_kernels.cu")
endif()

add_definitions(-DOPENCV)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
include_directories(
    ${SRC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

FILE(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/*.c)
LIST(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/server.c")
LIST(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/old.c")
LIST(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/cpu_gemm.c")
LIST(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/darknet.c")

if(CUDA_FOUND)
    cuda_add_executable(darknet 
        ${SRC_DIR}/main.cc
        ${SRC_DIR}/detector_class.cc
        ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
        ${PROJECT_SOURCE_DIR}/common/VectorCalculus.cc
        ${SRC_FILES} ${KERNELS}
    )
else()
    add_executable(darknet 
        ${SRC_DIR}/main.cc
        ${SRC_DIR}/detector_class.cc
        ${PROJECT_SOURCE_DIR}/common/ROSImageConverter.cc
        ${PROJECT_SOURCE_DIR}/common/VectorCalculus.cc
        ${SRC_FILES}
    )
endif()

target_link_libraries(darknet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CU_LIBRARIES}
)

set_target_properties(darknet
    PROPERTIES
    COMPILE_FLAGS ${DARKNET_COMPILE_FLAGS}
)

