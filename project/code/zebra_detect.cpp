/*********************************************************************************************************************
* 文件名称          zebra_detect.cpp
* 功能说明          斑马线检测模块 — 发车与停车判断
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-07-24        Cline                      first version
********************************************************************************************************************/

#include "zebra_detect.hpp"
#include <stdio.h>

// 斑马线计数（被其他模块读取）
uint8_t zebra_count = 0;

// 内部状态
static int  confirm_counter = 0;      // 连续检测帧计数
static int  cooldown_counter = 0;     // 冷却帧计数
static bool zebra_confirmed  = false;  // 当前是否已确认斑马线

void detect_zebra(const uint8_t* bin_img_data, int img_width, int img_height) {
    // ------ 冷却管理 ------
    if (cooldown_counter > 0) {
        cooldown_counter--;
        if (cooldown_counter == 0) {
            zebra_confirmed = false;   // 冷却结束，允许下一次检测
            confirm_counter = 0;
        }
        return;
    }

    // ------ 跳变计数 ------
    int transition_count = 0;
    int row_offset = ZEBRA_SCAN_Y * img_width;

    for (int x = 1; x < ZEBRA_IMAGE_WIDTH; x++) {
        uint8_t prev = bin_img_data[row_offset + x - 1];
        uint8_t curr = bin_img_data[row_offset + x];
        // 白→黑 跳变（255→0）
        if (prev == 255 && curr == 0) {
            transition_count++;
        }
    }

    // ------ 调试输出（每 10 帧打印跳变次数） ------
    static int debug_frame = 0;
    debug_frame++;
    if (debug_frame % 10 == 0) {
        printf("[ZEBRA] transitions=%d confirm=%d/%d zebra=%d\r",
               transition_count, confirm_counter, ZEBRA_CONFIRM_FRAMES, zebra_count);
        fflush(stdout);
    }

    // ------ 动态阈值：发车前严格(4)，发车后宽松(2) ------
    int threshold = (zebra_count == 0) ? ZEBRA_MIN_TRANSITIONS : 2;

    // ------ 连续帧确认 ------
    if (transition_count >= threshold) {
        confirm_counter++;
        if (confirm_counter >= ZEBRA_CONFIRM_FRAMES && !zebra_confirmed) {
            // 确认通过斑马线
            if (zebra_count < 2) {
                zebra_count++;
                printf("[ZEBRA] 检测到斑马线 #%d\n", zebra_count);
            }
            zebra_confirmed  = true;
            confirm_counter  = 0;
            cooldown_counter = ZEBRA_COOLDOWN_FRAMES;  // 进入冷却
        }
    } else {
        // 跳变不足，重置确认计数
        confirm_counter = 0;
    }
}