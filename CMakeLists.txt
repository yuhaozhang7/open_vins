cmake_minimum_required(VERSION 2.8.8)
project(openvins)
set(appname openvins)
set(CORE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ov_core/src)
set(MSCKF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ov_msckf/src)
include_directories(${SLAMBENCH_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_subdirectory(ov_core)
add_subdirectory(ov_msckf)

add_library(${appname}-original SHARED open_vins_library.cpp)

target_include_directories(${appname}-original PUBLIC ${CORE_INCLUDE_DIR} ${MSCKF_INCLUDE_DIR})
target_link_libraries(${appname}-original
        ov_msckf_lib
        ${thirdparty_libraries}
        ${OpenCV_LIBS}
        )

install(TARGETS ${appname}-original DESTINATION  lib/)
        
SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${WRAPPER_PATH}/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES INSTALL_RPATH "${LIB_PATH}")
    install(TARGETS ${library_name}-library  DESTINATION  lib/)

endfunction()

generate_slam_library(${appname}-original "-Wl,--disable-new-dtags")
