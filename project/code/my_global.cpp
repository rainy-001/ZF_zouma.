/*********************************************************************************************************************
* 文件名称          my_global.cpp
* 功能说明          全局资源管理模块实现 - 全局对象定义和线程回调函数实现
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-01-25        Assistant                  添加详细注释和分区优化
********************************************************************************************************************/

#include "my_global.hpp"
#include "niu_vision.hpp"

/* ================================================================================================================
 *                                           人机交互设备对象定义
 * ================================================================================================================ */
// 显示屏对象
zf_device_ips200 ips200;                                        // IPS200屏幕对象，240x320分辨率

// 按键和菜单系统
MyKey key_manager;                                              // 4按键管理器
MyMenu menu_system(&key_manager, &ips200);                      // 菜单系统（关联按键和屏幕）

/* ================================================================================================================
 *                                           图像采集与传输对象定义
 * ================================================================================================================ */
// USB摄像头
zf_device_uvc uvc;                                              // UVC摄像头对象，160x120@60fps
uint8_t* gray_img_ptr = nullptr;                                // 灰度图像数据指针（备用）
uint16_t* rgb_img_ptr = nullptr;                                // RGB图像数据指针（备用）


/* ================================================================================================================
 *                                           性能分析工具对象定义
 * ================================================================================================================ */
// 时间戳对象（用于性能测试）
TimerClockGetTime my_timer;                                     // 通用计时器（纳秒级精度）
TimerClockGetTime camera_timer;                                 // 摄像头帧率计时器
float image_proc_fps      = 0.0f;                               // 图像处理实际帧率
float image_proc_frame_ms = 0.0f;                               // 图像处理单帧耗时(ms)

/* ================================================================================================================
 *                                           速度控制系统变量定义
 * ================================================================================================================ */

// -------------------- 速度反馈变量（由编码器线程更新） --------------------
float right_speed = 0.0f;                                       // 右轮当前速度（只读）
float left_speed  = 0.0f;                                       // 左轮当前速度（只读）
                                                                // 更新周期：3ms（ENCODER_SAMPLING_PERIOD）

// -------------------- 速度设定变量（用户控制接口） --------------------
float target_speed_r = 0.0f;                                    // 右轮目标速度（可写）
float target_speed_l = 0.0f;                                    // 左轮目标速度（可写）
                                                                // 💡使用方式：直接赋值即可控制速度
                                                                // 示例：target_speed_r = 50.0f;
float cruising_speed = CRUISING_SPEED;                          // 巡航速度，修改该值会直接影响小车速度
float onto_control = 0;                                         // 方向PD控制量，跨线程传递参数

// -------------------- PWM输出变量（系统内部使用） --------------------
int16_t speed_to_pwm_r = 0;                                     // 右轮PWM输出值（PID计算结果）
int16_t speed_to_pwm_l = 0;                                     // 左轮PWM输出值（PID计算结果）

// // -------------------- PID控制器对象 --------------------
MyPID pid_r;                                                    // 右轮PID控制器
MyPID pid_l;                                                    // 左轮PID控制器
PDController pid_angle;                                         // 角度PID控制器
// ladrc控制方案--------------------------------------------------
SimpleMotorLADRC ladrc_left;
SimpleMotorLADRC ladrc_right;
SimpleMotorLADRC ladrc_onto_control;

// 初始化线程，可开始参数获取任务调度
zf_driver_pit_rt encoder_get;
zf_driver_pit_rt pid_control_thread;
zf_driver_pit_rt key_scan;
zf_driver_pit_rt lardc_control_thread;
zf_driver_pit_rt image_proc_thread;                             // 图像处理线程（20ms周期, 优先级96）
zf_driver_pit_rt display_bin_thread;                             // 二值图像显示线程（20ms/50Hz, 优先级94, 最低）


uint8_t onto_pd_control_enable = 0;                          // 角度PD控制使能标志（0=禁用，1=启用）

//---------------------- IMU ---------------------------
IMUHandler imu963r;                                             //陀螺仪与按键扫描在同一线程中执行，均为10ms
MadgwickAHRS ahrs(100);                                         //解算器，同步获取同步处理，注意检查性能消耗
float IMU_calibration = 180.0f/154.03f;                         //范围校准

//--------------------惯性导航控制器-----------------------------------
PathTracker path_tracker_component;                             //路径记录组件，内置里程计
AkimaInterpolator akima_component;                              //地图解算组件，将地图存储为指定名字的txt文件，格式：索引 x y

/* ================================================================================================================
 *                                           线程回调函数实现
 * ================================================================================================================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     KEY_SCAN_PERIOD (10ms)
// 线程优先级   98
// 备注信息     执行按键状态机扫描，检测按键事件
//-------------------------------------------------------------------------------------------------------------------
void key_scan_handler() //10ms
{
    key_manager.scan_keys();  // 执行按键扫描
    imu963r.update();         // 获取原始数据
   
    ahrs.updateIMU(
        imu963r.gyro[0]*IMU_calibration,  imu963r.gyro[2]*IMU_calibration, -imu963r.gyro[1]*IMU_calibration, 
        imu963r.acc[0],   imu963r.acc[2],  -imu963r.acc[1]   
    );
    // // 路径记录或定位
    // if(path_tracker_component.is_recording){
    //     path_tracker_component.record_sample(ahrs.getYaw());
    // }else if (path_tracker_component.is_reproduction)
    // {
    //     path_tracker_component.get_location(ahrs.getYaw());
    // }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     二值图像显示线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     DISPLAY_BIN_PERIOD (20ms, 50Hz)
// 线程优先级   94（最低优先级）
// 备注信息     从 bin_img_data 读取图像处理线程产出的二值化图像，
//              通过 ips200.show_gray_image() 显示在屏幕左上角 (0,0)-(160,120)
//-------------------------------------------------------------------------------------------------------------------
// [已禁用] 二值图像显示 — 避免 SPI 刷屏干扰实时控制线程
void display_bin_handler() {
    // IPS200 240×320, image_use / show = 188×120

    // 1. 上方：变换前原始二值图 (0,0)–(188,120)
    ips200.show_gray_image(0, 0, image_use[0], MT9V03X_W, MT9V03X_H);

    // 2. 下方：鸟瞰变换后二值图 (0,120)–(188,240)
    ips200.show_gray_image(0, MT9V03X_H, show[0], MT9V03X_W, MT9V03X_H);

    // 3. 下方叠加左边线（蓝色，鸟瞰坐标，y 偏移 MT9V03X_H）
    for (int i = 1; i < Lout_count && i < 100; i++) {
        ips200.draw_line((uint16)Luse_edge[i-1][0], (uint16)(MT9V03X_H + Luse_edge[i-1][1]),
                         (uint16)Luse_edge[i][0],   (uint16)(MT9V03X_H + Luse_edge[i][1]),   0x001F);
    }

    // 4. 下方叠加右边线（红色）
    for (int i = 1; i < Rout_count && i < 100; i++) {
        ips200.draw_line((uint16)Ruse_edge[i-1][0], (uint16)(MT9V03X_H + Ruse_edge[i-1][1]),
                         (uint16)Ruse_edge[i][0],   (uint16)(MT9V03X_H + Ruse_edge[i][1]),   0xF800);
    }

    // 5. 下方叠加中线（绿色）
    if (Lmidnum2 > 1) {
        for (int i = 1; i < Lmidnum2 && i < 50; i++) {
            ips200.draw_line((uint16)Lout_MID[i-1][0], (uint16)(MT9V03X_H + Lout_MID[i-1][1]),
                             (uint16)Lout_MID[i][0],   (uint16)(MT9V03X_H + Lout_MID[i][1]),   0x07E0);
        }
    }

    // 6. 状态信息
    static char buf[64];
    snprintf(buf, sizeof(buf), "FPS:%.1f err:%.1f dir:%d zL:%d zR:%d",
             image_proc_fps, error, xunxian_dir, ifzhi_L, ifzhi_R);
    ips200.show_string(0, MT9V03X_H * 2, buf);

    // 7. 刷新
    ips200.update();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像处理线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     IMAGE_PROC_PERIOD (20ms)
// 线程优先级   96
// 备注信息     固定周期执行视觉巡线管线:
//              等帧 → clone → image_proc() → 更新 onto
//              带防重入保护，上一帧未处理完则跳过本次
//-------------------------------------------------------------------------------------------------------------------
void image_proc_handler() {
    static bool in_progress = false;
    if (in_progress) return;
    in_progress = true;

    // 首帧启动 camera_timer
    static bool timer_started = false;
    static int  frame_count   = 0;
    if (!timer_started) { camera_timer.start(); timer_started = true; }

    if (uvc.wait_image_refresh() == 0) {
        // 获取灰度图
        uint8_t *gray = uvc.get_gray_image_ptr();
        if (gray) {
            // 牛爷爷全管线：Otsu → 最长白列 → 八邻域爬线 → 逆透视 → 滤波
            //               → 重采样 → 拐点+直道 → 中线 → 纯跟踪 → error
            niu_vision_pipeline(gray, UVC_WIDTH, UVC_HEIGHT);
        }
    }

    // 每 10 帧刷新一次帧率统计
    frame_count++;
    if (frame_count >= 10) {
        camera_timer.stop();
        long long elapsed = camera_timer.elapsed_ms();
        if (elapsed > 0) {
            image_proc_fps      = frame_count * 1000.0f / elapsed;
            image_proc_frame_ms = (float)elapsed / frame_count;
        }
        frame_count = 0;
        camera_timer.start();
    }

    in_progress = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器采集线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     ENCODER_SAMPLING_PERIOD (3ms)
// 线程优先级   99（最高优先级）
// 备注信息     ⚠️关键任务：读取编码器计数并映射为速度值，为PID控制提供实时反馈
//              此函数必须以最高优先级运行，确保速度采样的实时性和准确性
//-------------------------------------------------------------------------------------------------------------------
void encoder_get_count_handler()
{
    // -------------------- 核心功能：获取编码器速度 --------------------
    get_and_remap_speed(&left_speed, &right_speed, ENCODER_SAMPLING_PERIOD);

    // 同步到牛爷爷速度变量（供 madasudu() 使用）
    speedl = left_speed;
    speedr = right_speed;
    // 开启路径记录时进行里程计更新
    if(path_tracker_component.is_recording){
        path_tracker_component.right_tyre.update(((int16_t)right_speed));
        path_tracker_component.left_tyre.update(((int16_t)left_speed));
    }else if (path_tracker_component.is_reproduction)
    {
        path_tracker_component.right_tyre.update(((int16_t)right_speed));
        path_tracker_component.left_tyre.update(((int16_t)left_speed));
    }else{
        // path_tracker_component.right_tyre.reset();
        // path_tracker_component.left_tyre.reset();
    }
}

void hight_frequence_encoder_get_speed_handler(){
    lardc_get_speed(1,1024,0.035,0.001, &left_speed,&right_speed);
    if(onto_pd_control_enable==1){
        target_speed_l = target_speed_r = cruising_speed;//添加基准速度

        // speed_to_pwm_l = (int16_t)ladrc_left.calculatePWM(0, left_speed);
        // speed_to_pwm_r = (int16_t)ladrc_right.calculatePWM(-0, right_speed);

        speed_to_pwm_l = (int16_t)ladrc_left.calculatePWM(target_speed_l+onto_control, left_speed);
        speed_to_pwm_r = (int16_t)ladrc_right.calculatePWM(target_speed_r-onto_control, right_speed);
    }
    else{
        // speed_to_pwm_l = 0;
        // speed_to_pwm_r = 0;
    }
    motor_set_speed_ladrc(speed_to_pwm_l, speed_to_pwm_r);
    // motor_set_speed_ladrc(300, 300);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID速度控制线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     PID_CONTROL_PERIOD (15ms)
// 线程优先级   97
// 备注信息     执行PID速度闭环控制，根据目标速度和当前速度计算PWM输出
//              处理流程：读取target_speed → PID计算 → 输出PWM → 驱动电机
//              ⚠️注意：默认情况下，此线程启动后立即开始控制，若需电机停止，请设置target_speed=0
//-------------------------------------------------------------------------------------------------------------------
void pid_contol_handle()
{
    // 牛爷爷控制方案：纯跟踪 → error, PID 速度环 → madasudu
    if (onto_pd_control_enable == 1) {
        // 设置期望速度基准
        speedtobel = cruising_speed;
        speedtober = cruising_speed;

        // 调用牛爷爷纯跟踪：error 已在 niu_vision_pipeline 中更新
        // duty 已由 get_error() 计算好

        // 桥接：牛爷爷 duty → onto_control（差速转向）
        // duty 范围 ±70，映射到差速量
        onto_control = duty;

        // 牛爷爷速度 PID → pwml/pwmr
        madasudu();
    } else {
        onto_control = 0;
    }

    // 使用牛爷爷 madasudu 产出的 PWM（或 fallback 到原有 PID）
    if (onto_pd_control_enable == 1) {
        speed_to_pwm_l = (int16_t)pwml;
        speed_to_pwm_r = (int16_t)pwmr;
    } else {
        speed_to_pwm_l = 0;
        speed_to_pwm_r = 0;
    }

    motor_set_speed(speed_to_pwm_l, speed_to_pwm_r);
}

bool car_init(){
    // 导入透视变换矩阵
    save_per_map();
    // 导入基础参数并设置
    param_loading_from_file("/home/root/car_config.txt");
    param_print();
    // 1. 初始化显示屏
    printf("------------初始化显示屏-----------\n");
    ips200.init("/dev/fb0");
    ips200.clear();
    ips200.show_string(10, 10, "Camera & Encoder");
    ips200.show_string(10, 30, "System Ready");
    ips200.update();
    system_delay_ms(100);

    // 2. 初始化编码器（清零采样值）
    printf("------------初始化编码器------------\n");
    encoder1.clear_count();
    encoder2.clear_count();
    printf("编码器已清零\n");
    system_delay_ms(100);

    // 3.初始化DRV驱动
    printf("------------初始化DRV驱动------------\n");
    motor_init();
    left_speed = 0;
    right_speed = 0;
    motor_set_speed(left_speed, right_speed);

    // 4. 初始化摄像头
    printf("------------初始化摄像头------------\n");
    uvc.init("/dev/video0");

    // 5.初始化陀螺仪
    printf("------------初始化陀螺仪------------\n");
    imu963r.init();
    system_delay_ms(100);

    // 6. 初始化菜单系统
    printf("------------初始化菜单系统------------\n");
    // menu_system.init_menu();

    // printf("8. 初始化图像分类组件...\n");
    // if (!ncnn_classifier.init("/home/root/models/model_1/tiny_classifier_fp32.ncnn.param", "/home/root/models/model_1/tiny_classifier_fp32.ncnn.bin")) {
    //     printf("NCNN模型初始化失败！\n");
    //     return false;
    // }

    // TCP图像传输组件初始化 --- IGNORE ---
    // printf("9. TCP图像传输组件初始化...\n");
    // if (udp.init("192.168.43.94",8086)) {
    //     printf("NCNN模型初始化失败！\n");
    //     return false;
    // }

    // printf("9. TCP图像传输组件初始化...\n");
    // img_transmitter_init();

    if(control_model == 0){
        printf("pid control model");
        if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0)
        {
            printf("PID控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("pid control thread init successfully,period:%dms\n", PID_CONTROL_PERIOD);
        }

        if (encoder_get.init_ms(ENCODER_SAMPLING_PERIOD, encoder_get_count_handler, 99, true) != 0)
        {
            printf("编码器获取线程初始化失败\n");
            return false;
        }
        else
        {
            printf("encoder geting count thread init successfully,period: %dms\n", ENCODER_SAMPLING_PERIOD);
        }

    }else{
        if (lardc_control_thread.init_ms(LARDC_PERIOD, hight_frequence_encoder_get_speed_handler, 99, true) != 0)
        {
            printf("lardc控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("lardc control thread init successfully,period:%dms\n", LARDC_PERIOD);
        }

        // 方向PD
        if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0)
        {
            printf("PID控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("pid control thread init successfully,period:%dms\n", PID_CONTROL_PERIOD);
        }
    }
    
    if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true) != 0)
    {
        printf("定时器初始化失败\n");
        return false;
    }
    else
    {
        printf("key scaning thread init successfully,period: %dms\n", KEY_SCAN_PERIOD);
    }

    // 图像处理线程 (优先级96, 20ms周期, 50Hz)
    // 插入在方向PD(97)和IMU(95)之间，保证onto固定周期更新
    if (image_proc_thread.init_ms(IMAGE_PROC_PERIOD, image_proc_handler, 96, true) != 0)
    {
        printf("图像处理线程初始化失败\n");
        return false;
    }
    else
    {
        printf("image proc thread init successfully, period: %dms\n", IMAGE_PROC_PERIOD);
    }

    // [已禁用] 二值图像显示线程 — 避免 SPI 刷屏干扰实时控制
    if (display_bin_thread.init_ms(DISPLAY_BIN_PERIOD, display_bin_handler, 94, true) != 0)
    {
        printf("二值图像显示线程初始化失败\n");
        return false;
    }
    else
    {
        printf("binary display thread init successfully, period: %dms\n", DISPLAY_BIN_PERIOD);
    }

    return true;
}