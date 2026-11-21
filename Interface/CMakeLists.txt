# 稳定契约无源码、无平台宏，可同时被固件和主机替身使用。
add_library(signal_interface INTERFACE)
target_include_directories(signal_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
