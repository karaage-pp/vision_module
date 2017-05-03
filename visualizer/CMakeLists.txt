cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

set(MOC_HEADERS 
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/visualizer/visualizer.h")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
QT4_WRAP_CPP(MOC_SRCS ${MOC_HEADERS})

add_executable(visualizer 
    main.cc
    visualizer.cc
    ${MOC_SRCS}
)

target_link_libraries(visualizer
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
)

