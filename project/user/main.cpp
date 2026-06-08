#include "my_global.hpp"

int main()
{
    // 最小初始化：电机硬件（不启动控制线程）
    // printf("------------初始化电机驱动------------\n");
    // motor_init();
    // printf("电机驱动初始化完成\n");

    car_init();
    TCPClient client;
    if (!client.connect("192.168.79.38", 8086)) {
        return -1;
    }
     onto_pd_control_enable = 1;

    // ------------ 初始化陀螺仪 ------------
    // printf("------------初始化陀螺仪------------\n");
    // imu963r.init();
    // system_delay_ms(100);


    //菜单系统/定时系统初始化包含在car_init()中，确保在所有硬件初始化完成后再启动菜单系统和定时器,调参时注释此处
    // if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true) != 0)
    // {
    //     printf("定时器初始化失败\n");
    //     return false;
    // }
    // else
    // {
    //     printf("key scaning thread init successfully,period: %dms\n", KEY_SCAN_PERIOD);
    // }

    // menu_system.init();


    while (true) {
        if (client.receiveLine(received, 100)) {
            // 去除末尾换行符
            received.erase(received.find_last_not_of("\r\n") + 1);
            
            printf("收到: %s\n", received.c_str());
            
            if (received == "WRITE") {
                handleWriteCommand(client);
            }
            else if (received == "READ") {
                handleReadCommand();
            }
            else {
                // 尝试解析参数更新
               parseAndUpdateParameter(received);
            }
        }

        client.sendFormattedData("Onto_control,yaw,speed_l,speed_r,l_pwm,r_pwm:%f,%f,%f,%f,%d,%d\r\n",onto_control,ahrs.getYaw(),left_speed,right_speed,speed_to_pwm_l,speed_to_pwm_r);
        menu_system.menu_system();
        system_delay_ms(10);
    }

    return 0;
}
