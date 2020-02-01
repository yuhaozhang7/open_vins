cmake_minimum_required(VERSION 2.8.8)
project(open_vins)
set(CORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/ov_core/src)
add_subdirectory(ov_core)
add_subdirectory(ov_eval)
add_subdirectory(ov_msckf)

