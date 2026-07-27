/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_common_headfile
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
* 2026-07-25        Cline         极简版 - 仅保留WASD遥控所需模块
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_


//====================================================开源库公共层====================================================
#include "zf_common_font.hpp"
#include "zf_common_function.hpp"
#include "zf_common_fifo.hpp"
#include "zf_common_typedef.hpp"
//====================================================开源库公共层====================================================

//===================================================芯片外设驱动层===================================================
#include "zf_driver_delay.hpp"
#include "zf_driver_file_string.hpp"
#include "zf_driver_file_buffer.hpp"
#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_pit.hpp"
#include "zf_driver_adc.hpp"
#include "zf_driver_udp.hpp"
#include "zf_driver_tcp_client.hpp"
//===================================================芯片外设驱动层===================================================

//======== 以下设备驱动已注释（极简遥控版不需要）========
// #include "zf_device_imu.hpp"
// #include "zf_device_ips200_fb.hpp"
// #include "zf_device_uvc.hpp"
// #include "zf_device_dl1x.hpp"

//======== 以下组件层已注释（极简遥控版不需要）========
// #include "seekfree_assistant.hpp"
// #include "seekfree_assistant_interface.hpp"

//======== 以下外部组件库已注释（极简遥控版不需要）========
// #include "net.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <iostream>
// #include <fstream>
// #include <thread>
// #include <chrono>
// #include <atomic>

//===================================================用户自定义文件（极简版）===================================================
#include "motor.hpp"
#include "my_pid.hpp"
#include "remote_control.hpp"
//===================================================用户自定义文件===================================================
                                                                     




#endif