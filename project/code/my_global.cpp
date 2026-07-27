/*********************************************************************************************************************
* 文件名称          my_global.cpp
* 功能说明          全局资源管理模块实现（极简版 - 仅 WASD 键盘遥控）
* 适用平台          LS2K0300
********************************************************************************************************************/

#include "my_global.hpp"

/* ================================================================================================================
 *                                           变量定义
 * ================================================================================================================ */

// 编码器（定义在 motor.cpp 中，此处不再重复定义）
// 速度反馈
float right_speed = 0.0f;
float left_speed  = 0.0f;

// 速度设定
float target_speed_r = 0.0f;
float target_speed_l = 0.0f;
float cruising_speed  = 30.0f;     // 默认巡航速度 30
float onto_control   = 0.0f;

// PWM 输出




























int16_t speed_to_pwm_r = 0;
int16_t speed_to_pwm_l = 0;

// PID 控制器（使用默认构造，之后 init）
MyPID pid_r;
MyPID pid_l;
PDController pid_angle;

// 遥控对象
RemoteControl remote_ctrl;

// 线程对象
zf_driver_pit_rt encoder_get;
zf_driver_pit_rt pid_control_thread;

/* ================================================================================================================
 *                                           心跳超时变量
 * ================================================================================================================ */
static long long last_cmd_time_ms = 0;          // 上次收到指令的时间戳
static bool cmd_received_ever = false;          // 是否曾经收到过指令

/* ================================================================================================================
 *                                           线程回调函数实现
 * ================================================================================================================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器采集线程回调函数
// 调用周期     2ms
// 线程优先级   99（最高优先级）
//-------------------------------------------------------------------------------------------------------------------
void encoder_get_count_handler()
{
    get_and_remap_speed(&right_speed, &left_speed, ENCODER_SAMPLING_PERIOD);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID速度控制 + 遥控指令处理
// 调用周期     10ms
// 线程优先级   97
//-------------------------------------------------------------------------------------------------------------------
void pid_contol_handle()
{
    // ====== 1. 处理遥控指令 ======
    char cmd_char = 0;
    bool got_cmd = remote_ctrl.receive_command(cmd_char);

    if (got_cmd) {
        cmd_received_ever = true;
        
        switch (cmd_char) {
            case 'w': // 前进
                target_speed_r = target_speed_l = cruising_speed;
                onto_control = 0;
                printf("[CMD] 前进, speed=%.0f\n", cruising_speed);
                break;

            case 's': // 后退
                target_speed_r = target_speed_l = -cruising_speed;
                onto_control = 0;
                printf("[CMD] 后退, speed=%.0f\n", cruising_speed);
                break;

            case 'a': // 左转
                onto_control = +STEER_AMOUNT;
                printf("[CMD] 左转, steer=%.0f\n", STEER_AMOUNT);
                break;

            case 'd': // 右转
                onto_control = -STEER_AMOUNT;
                printf("[CMD] 右转, steer=%.0f\n", STEER_AMOUNT);
                break;

            case 'q': // 加速
                cruising_speed += SPEED_STEP;
                if (cruising_speed > MAX_CRUISING_SPEED) cruising_speed = MAX_CRUISING_SPEED;
                // 如果当前正在前进/后退，更新目标速度
                if (target_speed_r > 0 || target_speed_l > 0) {
                    target_speed_r = target_speed_l = cruising_speed;
                } else if (target_speed_r < 0 || target_speed_l < 0) {
                    target_speed_r = target_speed_l = -cruising_speed;
                }
                printf("[CMD] 加速, new cruising=%.0f\n", cruising_speed);
                break;

            case 'e': // 减速
                cruising_speed -= SPEED_STEP;
                if (cruising_speed < MIN_CRUISING_SPEED) cruising_speed = MIN_CRUISING_SPEED;
                if (target_speed_r > 0 || target_speed_l > 0) {
                    target_speed_r = target_speed_l = cruising_speed;
                } else if (target_speed_r < 0 || target_speed_l < 0) {
                    target_speed_r = target_speed_l = -cruising_speed;
                }
                printf("[CMD] 减速, new cruising=%.0f\n", cruising_speed);
                break;

            case ' ': // 停车
                target_speed_r = target_speed_l = 0;
                onto_control = 0;
                printf("[CMD] 停车\n");
                break;

            case 'x': // 急停
                target_speed_r = target_speed_l = 0;
                onto_control = 0;
                cruising_speed = 0;
                printf("[CMD] 急停!!\n");
                break;

            default:
                printf("[CMD] 未知指令: '%c' (0x%02X)\n", cmd_char, (unsigned char)cmd_char);
                break;
        }

        // 记录最后一次收到指令的时间
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        last_cmd_time_ms = ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
    }

    // ====== 2. 心跳超时检测 ======
    if (cmd_received_ever && remote_ctrl.is_connected()) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        long long now_ms = ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
        
        if (now_ms - last_cmd_time_ms > HEARTBEAT_TIMEOUT_MS) {
            target_speed_r = target_speed_l = 0;
            onto_control = 0;
            printf("[CMD] 心跳超时，自动停车\n");
            // 重置时间，避免反复打印
            last_cmd_time_ms = now_ms;
        }
    }

    // 如果客户端断开，清空目标
    if (!remote_ctrl.is_connected() && cmd_received_ever) {
        target_speed_r = target_speed_l = 0;
        onto_control = 0;
        // 重置标志，等待新连接
    }

    // ====== 3. PID 速度闭环 + 差速转向 ======
    // 差速转向: 左轮=target+onto, 右轮=target-onto
    speed_to_pwm_l = (int16_t)pid_l.control(target_speed_l + onto_control, left_speed);
    speed_to_pwm_r = (int16_t)pid_r.control(target_speed_r - onto_control, right_speed);
    
    motor_set_speed(speed_to_pwm_l, speed_to_pwm_r);
}

/* ================================================================================================================
 *                                           初始化函数
 * ================================================================================================================ */
bool car_init()
{
    printf("============ WASD 遥控小车初始化 ============\n");

    // 1. 初始化编码器（清零）
    printf("[Init] 初始化编码器...\n");
    encoder1.clear_count();
    encoder2.clear_count();
    printf("[Init] 编码器已清零\n");

    // 2. 初始化电机驱动
    printf("[Init] 初始化电机驱动...\n");
    motor_init();
    left_speed = 0;
    right_speed = 0;
    motor_set_speed(0, 0);
    printf("[Init] 电机驱动初始化完成\n");

    // 3. 初始化 PID 控制器（硬编码参数）
    //    参数来自 car_config.txt: p=2, t=0.01, i_gain=0.3, d=0.3, error_filter=0.02
    //    output_max=120, output_min=-120, integral_max=60, integral_min=-60
    printf("[Init] 初始化 PID 控制器...\n");
    pid_r.init(2.0f, 0.01f, 0.3f, 0.3f, 0.02f, 120.0f, -120.0f, 60.0f, -60.0f);
    pid_l.init(2.0f, 0.01f, 0.3f, 0.3f, 0.02f, 120.0f, -120.0f, 60.0f, -60.0f);
    printf("[Init] PID 初始化完成 (Kp=2.0, Ki_gain=0.3, Kd=0.3)\n");

    // 4. 启动编码器采集线程（2ms, 优先级99）
    if (encoder_get.init_ms(ENCODER_SAMPLING_PERIOD, encoder_get_count_handler, 99, true) != 0) {
        printf("[Init] 编码器线程初始化失败!\n");
        return false;
    }
    printf("[Init] 编码器采集线程启动 (周期%dms, 优先级99)\n", ENCODER_SAMPLING_PERIOD);

    // 5. 启动 PID 控制线程（10ms, 优先级97）
    if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0) {
        printf("[Init] PID控制线程初始化失败!\n");
        return false;
    }
    printf("[Init] PID控制线程启动 (周期%dms, 优先级97)\n", PID_CONTROL_PERIOD);

    // 6. TCP 遥控服务器（在主循环中非阻塞 accept/receive，不创建独立线程）
    printf("[Init] TCP遥控服务将在主循环中启动\n");

    printf("============ 初始化完成，等待遥控指令 ============\n");
    printf("PC端运行: ./car_controller <小车IP> %d\n", REMOTE_PORT);
    printf("按键说明: W前进 S后退 A左转 D右转 Q加速 E减速 空格停车 X急停\n");
    printf("============================================\n");

    return true;
}