#include "my_global.hpp"
#include "zebra_detect.hpp"

int main()
{
    car_init();
    onto_pd_control_enable = 0;  // 初始关闭方向PD，斑马线确认后才开启

    // ==================== 等待斑马线发车 ====================
    // car_init() 已启动所有控制线程，但 zebra_count=0 时 cruising_speed=0，车不动
    printf("\n========== 等待斑马线发车... ==========\n");
    while (zebra_count < 1) {
        usleep(5000);  // 等视觉线程检测到第一次斑马线
    }
    // 视觉确认斑马线 → 启用方向PD + 设置巡航速度，正式发车
    onto_pd_control_enable = 1;
    cruising_speed = CRUISING_SPEED;
    printf("========== 斑马线确认，发车！ ==========\n");
    // ====================================================

    while (true) {
        // 第二次斑马线 → 滑行后再停车
        if (zebra_count >= 2) {
            printf("========== 第二次斑马线，滑行中... ==========\n");
            // 继续跑 500ms（50 帧 × 10ms），让车完全越过斑马线
            for (int i = 0; i < 50; i++) {
                usleep(10000);
                menu_system.menu_system();
            }
            cruising_speed = 0;
            onto_pd_control_enable = 0;
            printf("========== 停车！ ==========\n");
            break;
        }

        menu_system.menu_system();
        usleep(5000);  // 5ms, 主循环≈200Hz
    }

    // 停车后保持静止
    motor_stop();
    while (true) {
        usleep(100000);
    }

    return 0;
}