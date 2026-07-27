cmake_minimum_required(VERSION 3.5.0)

# 1. 基础项目配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 2. 龙芯交叉编译参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# 交叉编译工具链
include(cross.cmake)

# 3. 头文件搜索路径（极简版：不需要 OpenCV/NCNN/设备驱动）
include_directories(
    ../user 
    ../code 
    ../../libraries/zf_common 
    ../../libraries/zf_driver
)

# 4. 显式收集源文件（不使用 aux_source_directory，避免编译旧模块）

# ---- project/user 目录 ----
list(APPEND DIR_SRCS ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

# ---- project/code 目录（仅遥控所需） ----
set(CODE_DIR "${CMAKE_CURRENT_LIST_DIR}/../code")
list(APPEND DIR_SRCS
    ${CODE_DIR}/my_global.cpp
    ${CODE_DIR}/my_pid.cpp
    ${CODE_DIR}/motor.cpp
    ${CODE_DIR}/remote_control.cpp
)

# ---- libraries/zf_common 目录（公共层） ----
set(ZF_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_common")
list(APPEND DIR_SRCS
    ${ZF_COMMON_DIR}/zf_common_fifo.cpp
    ${ZF_COMMON_DIR}/zf_common_font.cpp
    ${ZF_COMMON_DIR}/zf_common_function.cpp
)

# ---- libraries/zf_driver 目录（硬件抽象层） ----
set(ZF_DRIVER_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_driver")
list(APPEND DIR_SRCS
    ${ZF_DRIVER_DIR}/zf_driver_delay.cpp
    ${ZF_DRIVER_DIR}/zf_driver_encoder.cpp
    ${ZF_DRIVER_DIR}/zf_driver_gpio.cpp
    ${ZF_DRIVER_DIR}/zf_driver_pwm.cpp
    ${ZF_DRIVER_DIR}/zf_driver_pit.cpp
    ${ZF_DRIVER_DIR}/zf_driver_pit_fd.cpp
    ${ZF_DRIVER_DIR}/zf_driver_file_string.cpp
    ${ZF_DRIVER_DIR}/zf_driver_file_buffer.cpp
    ${ZF_DRIVER_DIR}/zf_driver_adc.cpp
    ${ZF_DRIVER_DIR}/zf_driver_udp.cpp
    ${ZF_DRIVER_DIR}/zf_driver_tcp_client.cpp
)

# 5. 禁用 TTF 字体（不需要显示屏）
add_definitions(-DWHETHER_USE_TTF=0)
message(STATUS "TTF Font Support: DISABLED (WASD遥控极简版)")

# 6. 生成可执行文件
add_executable(${PROJ_NAME} ${DIR_SRCS})

# 7. 链接系统库（不需要 OpenCV/NCNN）
target_link_libraries(${PROJ_NAME} 
    pthread 
    m 
    dl
)