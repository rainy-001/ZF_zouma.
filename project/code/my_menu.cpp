/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/
#include "my_menu.hpp"
#include "my_task_function.hpp"

// 全局菜单实例指针
MyMenu* g_menu_instance = nullptr;

Menu MyMenu::menu_table[] = {
    // ID, ParentID, Name, Function
    { 0 ,-1,"Main Menu",nullptr },
    { 1 ,0,"mode1",nullptr},
    { 2 ,0,"mode2",nullptr},
    { 3 ,0,"mode3",nullptr},
    { 4 ,0,"mode4",nullptr},
    { 5 ,0,"mode5",nullptr},
    { 6 ,0,"mode6",nullptr},
    { 7 ,0,"mode7",nullptr},
    { 8 ,1,"key_remap_test",MyMenu::key_remap_test},
    { 9 ,1,"imu_angle_display",MyMenu::imu_angle_display},
    { 10,2,"brushless_calibration",MyMenu::brushless_calibration},
    { 11,0,"motor_test",MyMenu::motor_test},
    { 12,0,"camera_test",MyMenu::camera_test}
};

//计算table大小
const int MyMenu::MENU_TABLE_SIZE = sizeof(MyMenu::menu_table) / sizeof(Menu);

MyMenu::MyMenu(MyKey* key_mgr, zf_device_ips200* ips_disp) 
    : key_manager(key_mgr), ips_display(ips_disp), mode_inter_flag(0) 
{
    //赋值操作，硬件初始化在init函数中进行
    g_menu_instance = this;
    current_menu = &menu_table[0]; 
}

void MyMenu::init(){
    g_menu_instance->ips_display->init(IPS_DEVICE_PATH);
    g_menu_instance->ips_display->clear();
    draw_menu(current_menu);
    g_menu_instance->ips_display->update();

}

// 根据 ID 查找菜单项指针
Menu* MyMenu::find_menu_by_id(int id) {
    if (id == -1) return nullptr;
    for (int i = 0; i < MENU_TABLE_SIZE; i++) {
        if (menu_table[i].id == id) return &menu_table[i];
    }
    return nullptr;
}

Menu* MyMenu::get_first_child(int parent_id) {
    for (int i = 0; i < MENU_TABLE_SIZE; i++) {
        if (menu_table[i].parent_id == parent_id) return &menu_table[i];
    }
    return nullptr;
}

Menu* MyMenu::menu_navigate(Menu *current, MenuAction action)
{
    if (!current) return &menu_table[0];
    int curr_idx = current - menu_table; // 计算当前指针在数组中的下标

    switch (action)
    {
    case MENU_UP:
    {
        // 向上找：在当前项之前，寻找第一个 parent_id 相同的项
        for (int i = curr_idx - 1; i >= 0; i--) {
            if (menu_table[i].parent_id == current->parent_id) {
                return &menu_table[i];
            }
        }
        break; // 没找到，维持现状
    }
    case MENU_DOWN:
    {
        // 向下找：在当前项之后，寻找第一个 parent_id 相同的项
        for (int i = curr_idx + 1; i < MENU_TABLE_SIZE; i++) {
            if (menu_table[i].parent_id == current->parent_id) {
                return &menu_table[i];
            }
        }
        break;
    }
    case MENU_OK:
    {
        // 1. 优先检查是否有子菜单
        Menu* child = get_first_child(current->id);
        if (child) {
            return child; // 发现子菜单，进入下一层，不改变 flag
        }
        
        // 2. 如果没有子菜单，说明这是一个功能叶子节点
        if (current->handler != nullptr) {
            //修改标志位，进入菜单交互
            this->mode_inter_flag = 1; 
            ips_display->clear();

        }
        return current;
    }
    case MENU_BACK:
    {
        // 返回父节点
        Menu* parent = find_menu_by_id(current->parent_id);
        return parent ? parent : current;
    }
    default:
        break;
    }
    return current;
}


void MyMenu::draw_menu(Menu *selected_menu)
{
    if (!selected_menu) return;

    // 1. 统计当前层级并找到选中项序号
    int p_id = selected_menu->parent_id;
    int total_in_level = 0;
    int selected_index_in_level = -1;

    for (int i = 0; i < MENU_TABLE_SIZE; i++)
    {
        if (menu_table[i].parent_id == p_id)
        {
            if (&menu_table[i] == selected_menu)
                selected_index_in_level = total_in_level;
            total_in_level++;
        }
    }

    // 2. 计算滚动分页逻辑 (保持原样)
    uint8 page_start = 0;
    if (selected_index_in_level >= MENU_MAX_ROW)
        page_start = selected_index_in_level - (MENU_MAX_ROW - 1);

    // 3. 执行绘制
    ips_display->clear();

    int current_item_count = 0; 
    int display_row = 0;         

    // 定义菜单布局参数
    const uint16 ITEM_HEIGHT = 28; // 由于使用了 TTF 24号字，行高建议设为 28-32

    for (int i = 0; i < MENU_TABLE_SIZE; i++)
    {
        if (menu_table[i].parent_id == p_id)
        {
            if (current_item_count >= page_start && display_row < MENU_MAX_ROW)
            {
                uint16 y_offset = display_row * ITEM_HEIGHT;

                if (&menu_table[i] == selected_menu)
                {
                    // --- 优化：绘制选中背景 ---
                    // 建议封装此功能，直接操作 buffer 以节省性能
                    // 这里直接调用你类的 pen_color 或者显式传参
                    ips_display->fill_rect(0, y_offset, ips_display->get_width(), ITEM_HEIGHT, MENU_SELECT_COLOR);

                    // --- 使用样式版 print：白色高亮文字 ---
                    // 假设选中时文字为白色，字号 24
                    ips_display->print(8, y_offset + 2, RGB565_WHITE, MENU_FONT_SIZE, menu_table[i].name);
                }
                else
                {
                    // --- 使用标准版 print：默认颜色文字 ---
                    // 假设非选中时使用默认画笔颜色
                    ips_display->print(8, y_offset + 2, menu_table[i].name);
                }
                display_row++;
            }
            current_item_count++;
        }
    }
    ips_display->update();
}

void MyMenu::menu_system(void)
{
    uint8 cl_action = key_manager->get_menu_action(); // 获取菜单动作
    // 非交互模式下无按键则跳过；交互模式下即使无按键也要刷新显示
    if (cl_action == WAITING && mode_inter_flag == 0) return;

    if (mode_inter_flag == 0)
    {
        uint8 pre_flag = mode_inter_flag; 
        
        // 1. 正常的菜单导航模式
        Menu* next_menu = menu_navigate(current_menu, (MenuAction)cl_action);
        
        // 如果菜单项发生了切换，刷新菜单界面
        if (next_menu != current_menu) {
            current_menu = next_menu;
            draw_menu(current_menu);
        }
        else if (mode_inter_flag != pre_flag && mode_inter_flag == 1) {
            if (current_menu->handler != nullptr) {
                // 此时 flag 刚变 1，通常需要立即执行一次 handler(ENTER) 来初始化功能界面
                current_menu->handler(cl_action);
            }
        }
    }
    else
    {   
        uint8 pre_flag = mode_inter_flag;
        
        // 2. 模式交互模式：直接调用当前菜单绑定的函数
        if (current_menu->handler != nullptr) {
            current_menu->handler(cl_action);
        } else {
            mode_inter_flag = 0; // 防御性逻辑
        }

        // 退出交互模式回到菜单导航时，立即刷新菜单页面
        if (pre_flag == 1 && mode_inter_flag == 0) {
            draw_menu(current_menu); 
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前菜单
// 参数说明 无
// 返回参数 Menu* 当前菜单指针
// 使用示例 Menu* current = menu_system.get_current_menu();
// 备注信息 获取当前选中的菜单项
//-------------------------------------------------------------------------------------------------------------------
Menu* MyMenu::get_current_menu(void)
{
    return current_menu;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置模式交互标志
// 参数说明 flag 标志值
// 返回参数 无
// 使用示例 menu_system.set_mode_inter_flag(1);
// 备注信息 设置是否进入模式交互状态
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::set_mode_inter_flag(uint8 flag)
{
    mode_inter_flag = flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取模式交互标志
// 参数说明 无
// 返回参数 uint8 当前标志值
// 使用示例 uint8 flag = menu_system.get_mode_inter_flag();
// 备注信息 获取当前模式交互状态
//-------------------------------------------------------------------------------------------------------------------
uint8 MyMenu::get_mode_inter_flag(void)
{
    return mode_inter_flag;
}

// 菜单功能函数，依照函数功能，注意维护id-------------------------------------------------------------------
void MyMenu::menu_mode_0(uint8 cl_action)
{
    if(g_menu_instance==nullptr) return;
    // 模式0的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式0
        break;
    case 1: // 上键
        break;
    case 2: // 下键
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }  

    tracking();
}

void MyMenu::menu_mode_1(uint8 cl_action)
{
    if(g_menu_instance==nullptr) return;
    // 模式1的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式1
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    // 显示模式1信息
    // send_picture_to_Serve();
    // tracking();
}

void MyMenu::menu_mode_2(uint8 cl_action)
{
    // 模式2的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式2
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
}

void MyMenu::menu_mode_3(uint8 cl_action)
{
    // 模式3的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式3
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式3信息
    g_menu_instance->ips_display->clear();
    g_menu_instance->ips_display->show_string(0, 0, "Mode 3 Active");
}

void MyMenu::key_remap_test(uint8 cl_action)
{
    static int temp_test = 0;
    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: 
        temp_test++;
        break;
    case 1:
        temp_test--;
        break;
    case 2:
        temp_test = 0;
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式4信息
    g_menu_instance->ips_display->clear();
    g_menu_instance->ips_display->print(0, 0, "this is model 4!");
    g_menu_instance->ips_display->print(0, 16, "key remap test:1.+;2.-;");
    g_menu_instance->ips_display->print(0, 32, "     3.set 0;4.return;");
    g_menu_instance->ips_display->print(0,3*16,"-------\ntest num:%d\n--------",temp_test);
    g_menu_instance->ips_display->print(0,7*16,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24,DEFAULT_PENCOLOR,36,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24+36,DEFAULT_PENCOLOR,48,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24+36+48,DEFAULT_PENCOLOR,64,"你好，世界");
    g_menu_instance->ips_display->update();
    
}

void MyMenu::menu_mode_6(uint8 cl_action)
{

    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: // 切换键
        break;
    case 1: // +
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
}


void MyMenu::imu_angle_display(uint8 cl_action)
{
    // IMU角度实时显示模式
    switch (cl_action)
    {
    case 0: // OK键 — 重置航向角归零
        ahrs.reset();
        break;

    case 3: // BACK键 — 退出，返回菜单
        g_menu_instance->mode_inter_flag = 0;
        return; // 退出时不需要再刷屏，菜单层会接管

    default:
        break;
    }

    // --- 刷新 IMU 实时数据到屏幕 ---
    zf_device_ips200* disp = g_menu_instance->ips_display;

    disp->clear();

    // ▎标题行 (字号 20，略小于菜单字号，节省纵向空间)
    disp->print(0, 0, RGB565_WHITE, 20.0f, "=== IMU Monitor ===");

    // 横线分隔
    disp->draw_line(0, 22, disp->get_width(), 22, RGB565_BLACK);

    // ▎欧拉角 (Roll / Pitch / Yaw) — 姿态核心数据
    disp->print(0, 28,  "Roll :%+8.2f deg", ahrs.getRoll());
    disp->print(0, 50,  "Pitch:%+8.2f deg", ahrs.getPitch());
    disp->print(0, 72,  "Yaw  :%+8.2f deg", ahrs.getYaw());

    // 分隔线
    disp->draw_line(0, 96, disp->get_width(), 96, RGB565_BLACK);

    // ▎陀螺仪角速度 (经低通滤波后的物理值)
    disp->print(0, 102, "--- Gyro (deg/s) ---");
    disp->print(0, 124, "GX:%+7.2f  GY:%+7.2f",
                (double)imu963r.gyro[0], (double)imu963r.gyro[1]);
    disp->print(0, 146, "GZ:%+7.2f",
                (double)imu963r.gyro[2]);

    // 分隔线
    disp->draw_line(0, 172, disp->get_width(), 172, RGB565_BLACK);

    // ▎加速度计 (含重力分量)
    disp->print(0, 178, "--- Acc (m/s^2) ---");
    disp->print(0, 200, "AX:%+6.2f  AY:%+6.2f",
                (double)imu963r.acc[0], (double)imu963r.acc[1]);
    disp->print(0, 222, "AZ:%+6.2f",
                (double)imu963r.acc[2]);

    // ▎底部操作提示
    disp->draw_line(0, 252, disp->get_width(), 252, RGB565_BLACK);
    disp->print(0, 258, "[OK] Reset Yaw  [BACK] Return");

    // 推向屏幕
    disp->update();
}

// 惯性导航地图记录界面
void MyMenu::get_map(uint8 cl_action) {
    if (!path_tracker_component.is_recording) {
        g_menu_instance->ips_display->show_string(1,1,"1, record");
        g_menu_instance->ips_display->show_string(1,1+1*16,"2, clear_map");
        g_menu_instance->ips_display->show_string(1,1+2*16,"3, insert_map");
        g_menu_instance->ips_display->show_string(1,1+4*16,"4, return");
        g_menu_instance->ips_display->update();
    }

    switch (cl_action) {
        case 0: // 记录开关
            if (!path_tracker_component.is_recording) {
                ahrs.reset(); // 重置姿态，确保航向角从0开始
                system_delay_ms(500);
                path_tracker_component.start_remember();
            } else {
                path_tracker_component.stop_remember(false);
            }
            break;

        case 1: // 清空内存中的地图数据
            path_tracker_component.stop_remember(true);
            break;

        case 2: // 转换、平滑并保存地图
            if(!path_tracker_component.is_recording){
                if (path_tracker_component.current_index > 5) {
                    // 1. 先进行高斯滤波，消除编码器和传感器的原始噪点
                    path_tracker_component.apply_gaussian_filter(1.2f, 5);

                    // 2. 将 PathTracker 中的原始点提取并转换为“里程-坐标”序列
                    std::vector<double> s_list; // 累计里程（自变量）
                    std::vector<double> x_list; // X 坐标
                    std::vector<double> y_list; // Y 坐标
                    double total_s = 0;

                    // 插入起点
                    s_list.push_back(0);
                    x_list.push_back(path_tracker_component.path_array[0].x);
                    y_list.push_back(path_tracker_component.path_array[0].y);

                    // 遍历记录的所有点，计算累计里程 s
                    for (int i = 1; i < path_tracker_component.current_index; i++) {
                        double dx = path_tracker_component.path_array[i].x - path_tracker_component.path_array[i-1].x;
                        double dy = path_tracker_component.path_array[i].y - path_tracker_component.path_array[i-1].y;
                        total_s += std::sqrt(dx * dx + dy * dy);

                        s_list.push_back(total_s);
                        x_list.push_back(path_tracker_component.path_array[i].x);
                        y_list.push_back(path_tracker_component.path_array[i].y);
                    }

                    // 3. 使用 Akima 进行插值计算
                    AkimaInterpolator x_engine, y_engine;
                    if (x_engine.compute(s_list, x_list) && y_engine.compute(s_list, y_list)) {
                        // 4. 等距采样并保存为文件
                        AkimaInterpolator::save_as_tracking_map(
                            x_engine, 
                            y_engine, 
                            total_s, 
                            10, 
                            "tracking_map.txt"
                        );
                        AkimaInterpolator::convert_txt_to_bin("tracking_map.txt","tracking_map.bin");
                        // 可以在屏幕上提示 "Save OK!"
                        printf("------save successful------\n");
                    }
                }
                else
                {
                     g_menu_instance->ips_display->show_string(1,1,"map too short");
                }
                
            }
            break;

        case 3: // 返回键
            // 重置所有状态
            ahrs.reset();
            path_tracker_component.reset();
            
            g_menu_instance->mode_inter_flag = 0;
            break;
    }
}

// 惯导路径复现
void MyMenu::tracking_by_map(uint8 cl_action){
    if (!path_tracker_component.is_reproduction) {
        g_menu_instance->ips_display->show_string(1,1,"1, reproduction");
        g_menu_instance->ips_display->show_string(1,1+1*16,"2, check_and_load_map");
        g_menu_instance->ips_display->show_string(1,1+2*16,"3, reset_status");
        g_menu_instance->ips_display->show_string(1,1+4*16,"4, return");
        g_menu_instance->ips_display->update();
    }else{
        path_tracker_component.find_closest_index();
        path_tracker_component.get_look_ahead_point(3);
        // path_tracker_component.calculate_target_yaw();
    }

    switch (cl_action)
    {
    case 0: // 开始复现
        // 开始前检查一下内存里的 map 是否为空
        if (path_tracker_component.tracking_map.empty()) {
            printf("Error: Map not loaded. Press 'Confirm' first.\n");
        } else {
            printf("Resuming path following...\n");
            ahrs.reset();
            path_tracker_component.reset();
            // 切换小车状态机到自动驾驶模式

            path_tracker_component.is_reproduction = true;
            
        }
        break;

    case 1: // 确认地图并加载地图
        printf("Checking map file...\n");
        
        // 尝试从磁盘加载二进制地图到内存
        if (path_tracker_component.load_binary_map("tracking_map.bin")) {
            // 加载成功后，打印地图信息
            printf("Map is available!\n");
            printf("Points: %d\n", (int)path_tracker_component.tracking_map.size());
            // 重置搜索起始点，确保从头开始
            path_tracker_component.last_closest_idx = 0;
             
        } else {
            // 加载失败，打印具体原因
            printf("Can't loading map");
        }
        break;

    case 2: // 重置位置，角度，路径点
        path_tracker_component.reset(); // 清除坐标累积
        path_tracker_component.last_closest_idx = 0; // 重置搜索索引
        ahrs.reset(); //解算器重置

        printf("Position and Index Reset Done.\n");
        break;

    case 3: // 返回键
        //清除所有状态
        path_tracker_component.reset();
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;

    default:
        break;
    }
}

// 电机测试菜单
void MyMenu::motor_test(uint8 cl_action)
{
    static int16_t test_speed = 20;      // 默认测试速度
    static bool motor_enabled = false;   // 电机使能状态

    // 按键处理
    switch (cl_action)
    {
    case 0: // OK键 — 切换电机使能
        motor_enabled = !motor_enabled;
        if (!motor_enabled) {
            motor_set_speed(0, 0);
        }
        break;

    case 1: // 上键 — 加速
        test_speed += 5;
        if (test_speed > 120) test_speed = 120;
        break;

    case 2: // 下键 — 减速
        test_speed -= 5;
        if (test_speed < -120) test_speed = -120;
        break;

    case 3: // 返回键 — 停止电机并退出
        motor_set_speed(0, 0);
        motor_enabled = false;
        test_speed = 20;  // 重置默认速度
        g_menu_instance->mode_inter_flag = 0;
        return;

    default:
        break;
    }

    // 根据使能状态控制电机
    if (motor_enabled) {
        motor_speed_test(test_speed, test_speed);
    }

    // 刷新显示
    g_menu_instance->ips_display->clear();
    g_menu_instance->ips_display->print(0, 0,
        "=== Motor Test ===");
    g_menu_instance->ips_display->print(0, 20,
        "Target: %d", test_speed);
    g_menu_instance->ips_display->print(0, 40,
        "Status: %s", motor_enabled ? "RUN" : "STOP");
    g_menu_instance->ips_display->print(0, 60,
        "L_speed: %.1f", left_speed);
    g_menu_instance->ips_display->print(0, 80,
        "R_speed: %.1f", right_speed);
    g_menu_instance->ips_display->print(0, 110,
        "UP/DOWN:+-5 OK:on/off");
    g_menu_instance->ips_display->print(0, 130,
        "BACK: stop & return");
    g_menu_instance->ips_display->update();
}

// 摄像头实时预览菜单
void MyMenu::camera_test(uint8 cl_action)
{
    static bool camera_inited = false;

    // 首次进入时初始化摄像头
    if (!camera_inited) {
        printf("------------初始化摄像头------------\n");
        uvc.init("/dev/video0");
        camera_inited = true;
    }

    // 按键处理
    switch (cl_action)
    {
    case 3: // BACK键 — 退出，返回菜单
        g_menu_instance->mode_inter_flag = 0;
        return;

    default:
        break;
    }

    // --- 实时刷新摄像头画面到屏幕 ---
    zf_device_ips200* disp = g_menu_instance->ips_display;

    // 使用 RGB565 直接获取（memcpy 逐行拷贝，比灰度逐像素转换快得多）
    rgb_img_ptr = uvc.get_rgb_image_ptr();
    int refresh_ret = uvc.wait_image_refresh();  // 0=成功, -1=失败

    // 只清标题栏和底部栏，不清图像区域（省去 76,800 像素的冗余填充）
    disp->fill_rect(0, 0, disp->get_width(), 24, DEFAULT_BGCOLOR);
    disp->fill_rect(0, 290, disp->get_width(), disp->get_height() - 290, DEFAULT_BGCOLOR);

    // 标题行
    disp->print(0, 0, RGB565_WHITE, 20.0f, "=== Camera Test ===");

    if (refresh_ret == 0 && rgb_img_ptr != nullptr) {
        // RGB565 图像：逐行 memcpy，远快于 gray→RGB565 逐像素转换
        disp->show_rgb_image(0, 24, rgb_img_ptr, UVC_WIDTH, UVC_HEIGHT);
    } else {
        disp->print(0, 100, "No camera frame...");
    }

    // 底部操作提示
    disp->draw_line(0, 290, disp->get_width(), 290, RGB565_BLACK);
    disp->print(0, 294, "[BACK] Return");

    disp->update();
}

// 无刷电调校准菜单
void MyMenu::brushless_calibration(uint8 cl_action)
{
    switch (cl_action)
    {
    case 0:
        esc_set_power(100);
        break;
    case 1:
        esc_set_power(0);
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        esc_set_power(0);    // 安全处理，关闭无刷电机
        break;
    default:
        break;
    }
    g_menu_instance->ips_display->show_string(1,1,"1.full power");
    g_menu_instance->ips_display->show_string(1,1+16*1,"2.min power");
    g_menu_instance->ips_display->update();

}