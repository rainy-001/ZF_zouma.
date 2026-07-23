#include "my_global.hpp"

int main()
{
    // ==================== 等待按键发车（放在 car_init 之前, 所有线程尚未启动, 电机绝不可能转） ====================
    printf("\n========== 按 OK 键发车 ==========\n");

    while (1) {
        key_manager.scan_keys();                        // 手动扫描按键（线程尚未启动）
        MenuAction action = key_manager.get_menu_action();
        if (action == MENU_OK) break;
        usleep(10000);                                  // 10ms
    }

    printf("========== 发车！==========\n");
    // ====================================================

    car_init();
    onto_pd_control_enable = 1;

    while (true) {
        menu_system.menu_system();
        usleep(5000);  // 5ms, 主循环≈200Hz
    }

    return 0;
}