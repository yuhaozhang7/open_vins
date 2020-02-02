cmake_minimum_required(VERSION 2.8.8)
project(open_vins)
set(CORE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ov_core/src)
set(MSCKF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ov_msckf/src)
add_subdirectory(ov_core)
add_subdirectory(ov_msckf)

add_library(${appname}-original SHARED open_vins_library.cpp)

target_include_directories(${appname}-original PUBLIC ${CORE_INCLUDE_DIR} ${MSCKF_INCLUDE_DIR})
target_link_libraries(${appname}-original
        ov_msckf_lib
        ${thirdparty_libraries}
        ${OpenCV_LIBS}
        )

generate_slam_library(${PROJECT_NAME}-original "")
