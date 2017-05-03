cmake_minimum_required(VERSION 2.8.3)
project(vision_module)

set(IMAGE_OUTPUT_DIR "image")
set(PYFILE_DIR "pyfiles")
set(MODEL_DIR "model")
set(FACEMODEL_DIR "facemodel")
set(FACEDATA_DIR "facedata")

#画像サイズ変えたいときはここを変える
set(RGBD_IMAGE_WIDTH "640")
set(RGBD_IMAGE_HEIGHT "480")
##

math(EXPR RGBD_IMAGE_SIZE "${RGBD_IMAGE_WIDTH} * ${RGBD_IMAGE_HEIGHT}")
set(WORKSPACE "${PROJECT_SOURCE_DIR}/vision_pack")
set(PYWORKSPACE "${WORKSPACE}/${PYFILE_DIR}")


if(NOT EXISTS ${WORKSPACE}/${IMAGE_OUTPUT_DIR})
    file(MAKE_DIRECTORY ${WORKSPACE}/${IMAGE_OUTPUT_DIR})
endif()
foreach(var ${PYFILE_DIR} ${MODEL_DIR} ${FACEMODEL_DIR} ${FACEDATA_DIR})
    if(NOT EXISTS ${WORKSPACE}/${var})
        message(FATAL_ERROR
            "No such directory ${WORKSPACE}/${var}")
    endif()
endforeach()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    pcl_ros
    dynamic_reconfigure
    cv_bridge
    message_generation
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_PATH}
)

add_message_files(
    FILES
    Vector.msg
    NBest.msg
    VisionOrder.msg
    ImageInfo.msg
    PlaneData.msg
    PlaneInfo.msg
    ObjectData.msg
    ObjectInfo.msg
    FaceData.msg
    FaceInfo.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
    )

file(GLOB dcfgs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/cfg/*.cfg)
generate_dynamic_reconfigure_options(
    ${dcfgs}
)

file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/*)
list(REMOVE_ITEM children "CMakeLists.txt")
list(REMOVE_ITEM children "package.xml")
list(REMOVE_ITEM children "plugin.xml")
list(REMOVE_ITEM children "include")
list(REMOVE_ITEM children "cfg")
list(REMOVE_ITEM children "msg")
list(REMOVE_ITEM children "launch")
foreach(child ${children})
    if(EXISTS ${PROJECT_SOURCE_DIR}/${child}/CMakeLists.txt)
        message(STATUS "vision_module: add_subdirectory(${child})")
        add_subdirectory(${child})
        #        add_dependencies(${child} ${PROJECT_NAME}_gencfg)
        set_target_properties(${child}
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY
            ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
    endif()
endforeach()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs cv_bridge message_runtime
#  DEPENDS system_lib
)
