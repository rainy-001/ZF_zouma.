/*********************************************************************************************************************
* 文件名称          zebra_detect.hpp
* 功能说明          斑马线检测模块 — 发车与停车判断
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-07-24        Cline                      first version
********************************************************************************************************************/

#ifndef __ZEBRA_DETECT_HPP__
#define __ZEBRA_DETECT_HPP__

#include <stdint.h>

// 斑马线检测参数
#define ZEBRA_SCAN_Y             110        // 扫描行（图像底部，120行中倒数第10行）
#define ZEBRA_IMAGE_WIDTH        160        // 图像宽度
#define ZEBRA_MIN_TRANSITIONS    4          // 最少黑白跳变次数
#define ZEBRA_CONFIRM_FRAMES     10         // 连续确认帧数（~200ms @50Hz）
#define ZEBRA_COOLDOWN_FRAMES    100        // 检测后冷却帧数（~2秒），防止同一段斑马线重复计数

// 斑马线检测状态
extern uint8_t zebra_count;                // 0=未检测, 1=首次(发车), 2=第二次(停车)

/**
 * @brief 斑马线检测主函数，每帧调用
 * @param bin_img_data  二值化图像数据 (0/255, 160x120)
 * @param img_width     图像宽度
 * @param img_height    图像高度
 */
void detect_zebra(const uint8_t* bin_img_data, int img_width, int img_height);

#endif // __ZEBRA_DETECT_HPP__