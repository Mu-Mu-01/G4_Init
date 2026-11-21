cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME EPI_cnt5_2026.11.21)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# 默认使用 Project 目录下共享的 CMSIS-DSP 源码，可通过 CMake 缓存变量覆盖。
get_filename_component(CMSIS_DSP_DEFAULT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../CMSIS-DSP-main" ABSOLUTE)
set(CMSIS_DSP_ROOT "${CMSIS_DSP_DEFAULT_ROOT}" CACHE PATH "CMSIS-DSP 源码根目录")
if(NOT EXISTS "${CMSIS_DSP_ROOT}/Source/CMakeLists.txt")
    message(FATAL_ERROR "找不到 CMSIS-DSP 源码，请设置 CMSIS_DSP_ROOT 指向 CMSIS-DSP 根目录：${CMSIS_DSP_ROOT}")
endif()

# 复用 CubeMX 工程的 CMSIS-Core 头文件。DSP 静态库继承工具链的 CPU 和浮点选项。
set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
set(DISABLEFLOAT16 ON)
add_subdirectory("${CMSIS_DSP_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/cmsis-dsp")
# 消费者和库使用一致的半精度接口配置。GCC 的数学库随 DSP 目标传播，供最终链接使用。
target_compile_definitions(CMSISDSP INTERFACE DISABLEFLOAT16)
if(TOOLCHAIN_LINK_LIBRARIES)
    target_link_libraries(CMSISDSP PRIVATE ${TOOLCHAIN_LINK_LIBRARIES})
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP

    # Add user defined libraries
)
