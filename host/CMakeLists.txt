cmake_minimum_required(VERSION 3.21)

project(xrtdev-host)
set(CMAKE_CXX_STANDARD 17)

# Default Install Directory
set(XRT_DIR "/opt/xilinx/xrt" CACHE STRING "Xilinx XRT Directory")


add_library(xrt INTERFACE)
target_include_directories(xrt INTERFACE ${XRT_DIR}/include)
target_link_libraries(xrt INTERFACE ${XRT_DIR}/lib/libxrt_coreutil_static.a)

add_library(host INTERFACE)
target_include_directories(host INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(host INTERFACE xrt)


list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
include(exec)
exec_dir(${CMAKE_CURRENT_SOURCE_DIR}/tests "tests." ${CMAKE_CURRENT_BINARY_DIR}/tests true false host)
