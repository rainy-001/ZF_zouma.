# 智能车菜单系统调试全记录

> 日期：2026-06-07
> 项目：ZF_zouma 智能车
> 平台：LS2K0300 / Buildroot

---

## 目录

1. [问题一：IMU 角度显示全为 0](#一问题一imu-角度显示全为-0)
2. [问题二：IMU 界面不实时刷新](#二问题二imu-界面不实时刷新)
3. [问题三：静止时角度漂移](#三问题三静止时角度漂移)
4. [功能：摄像头预览挂载到菜单](#四功能摄像头预览挂载到菜单)
5. [问题四：摄像头显示 "No camera frame"](#五问题四摄像头显示-no-camera-frame)
6. [涉及文件清单](#六涉及文件清单)

---

## 一、问题一：IMU 角度显示全为 0

### 现象

执行菜单挂载的 IMU 测试函数（`imu_angle_display`），屏幕上 Roll/Pitch/Yaw/Gyro/Acc 全部显示 0。

### 根因

`main.cpp:10` 中 `car_init()` 被注释掉，IMU 硬件从未初始化。

```cpp
// main.cpp
int main() {
    motor_init();
    // car_init();   // ← 被注释！imu963r.init() 从未执行
    ...
    key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true);
}
```

### 数据流追踪

```
car_init() 被注释
  └─ imu963r.init() 未调用
       └─ zf_device_imu::init() 未执行
            ├─ IIO event 文件未打开，硬件未初始化
            ├─ imu_type 保持 DEV_NO_FIND (0)
            └─ 加速度/陀螺仪 sysfs 文件句柄未打开

key_scan_handler() 每 10ms 执行:
  └─ imu963r.update()
       └─ imu_dev.get_acc_x() → imu_read_fd_data(fd=-1)
            └─ if (imu_type == DEV_NO_FIND || fd < 0) return 0;  ← 命中！
       └─ 所有传感器数据为 0
  └─ ahrs.updateIMU(0, 0, 0, 0, 0, 0)
       └─ 加速度全零 → 跳过反馈修正 (MadgwickAHRS.cpp:177)
       └─ 陀螺全零 → 四元数不变化
       └─ Roll/Pitch/Yaw 始终为 0
```

### 修复

**文件**：`project/user/main.cpp`

在 `key_scan` 线程启动前添加 IMU 初始化：

```cpp
// ------------ 初始化陀螺仪 ------------
printf("------------初始化陀螺仪------------\n");
imu963r.init();          // 打开 IIO sysfs 文件，初始化硬件
system_delay_ms(100);    // 等待硬件稳定

if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true) != 0)
```

`imu963r.init()` 内部流程：
1. `zf_device_imu::init()` → 向 `/sys/bus/iio/devices/iio:device1/events/in_voltage_change_en` 写 `'1'` 触发硬件初始化
2. 读取设备型号（IMU660RA/B 或 IMU963RA）
3. 打开加速度计/陀螺仪 IIO sysfs 文件句柄
4. `calibrate_offsets(100)` — 100 次采样计算陀螺零偏

---

## 二、问题二：IMU 界面不实时刷新

### 现象

进入 IMU 显示界面后，屏幕静止不刷新。只有按下按键（如 OK）时才刷新一次。

### 根因

`my_menu.cpp:191` — 无按键时直接返回，交互模式下的 handler 不会被调用来刷新屏幕。

```cpp
void MyMenu::menu_system(void) {
    uint8 cl_action = key_manager->get_menu_action();
    if (cl_action == WAITING) return;  // ← 无按键直接返回！
    ...
}
```

### 数据流追踪

```
main() 循环每 10ms:
  └─ menu_system.menu_system()
       └─ key_manager->get_menu_action()
            ├─ 有按键 → 返回 MENU_OK(2)/MENU_BACK(3) 等
            └─ 无按键 → 返回 WAITING(4)
                 └─ if (cl_action == WAITING) return;  ← 直接返回！
                 └─ imu_angle_display() 不被调用
                 └─ 屏幕不刷新
```

### 修复

**文件**：`project/code/my_menu.cpp`

```cpp
// 修改前：
if (cl_action == WAITING) return;

// 修改后：
if (cl_action == WAITING && mode_inter_flag == 0) return;
```

逻辑变化：

| `cl_action` | `mode_inter_flag` | 修改前 | 修改后 |
|---|---|---|---|
| `WAITING` (无按键) | `0` (菜单导航) | return（不变） | return（不变） |
| `WAITING` (无按键) | `1` (交互模式) | **return（不刷新❌）** | **继续执行 handler（刷新✅）** |
| 有按键 | 任意 | 执行 handler | 执行 handler |

---

## 三、问题三：静止时角度漂移

### 现象

小车静止不动，但屏幕上 Pitch/Roll/Yaw 角度值持续缓慢变化。

### 根因分析

存在 4 个导致漂移的因素：

| # | 问题 | 位置 | 影响 |
|---|------|------|------|
| 1 | 零偏校准时间太短 | `IMU963R.cpp:17` — 只采样 100 次 × 2ms = **200ms** | 零偏不准 → 陀螺静态输出非零 → 积分漂移 |
| 2 | Madgwick β 值过低 | `my_global.cpp:80` — `MadgwickAHRS ahrs(100)` β=**0.1** | 加速度计修正力太弱，不足以纠正陀螺漂移 |
| 3 | 陀螺低通滤波太弱 | `IMU963R.hpp:24` — `alpha_gyro=0.5` | 高频噪声透过，积分后随机游走 |
| 4 | Yaw 无绝对参考 | `key_scan_handler` 调用 `updateIMU()`（6 轴） | **无磁力计，Yaw 漂移物理上不可避免** |

核心矛盾：`beta=0.1` → Madgwick 滤波器 **90% 信任陀螺仪，仅 10% 信任加速度计**。零偏不准时，错误陀螺数据主导姿态估计，重力参考无法有效纠偏。

### 关键代码位置

**校准（太短）** — `IMU963R.cpp:17`：
```cpp
imu_device_type_enum IMUHandler::init(void) {
    imu_device_type_enum type = imu_dev.init();
    if (type != DEV_NO_FIND) {
        calibrate_offsets(100);  // 100次 × 2ms = 200ms
    }
    return type;
}
```

**Madgwick 参数** — `my_global.cpp:80`：
```cpp
MadgwickAHRS ahrs(100);  // sampleFreq=100Hz, beta=0.1（默认）
```

**数据更新** — `my_global.cpp:99-107`：
```cpp
void key_scan_handler() {
    key_manager.scan_keys();
    imu963r.update();  // 低通滤波后得到 gyro[], acc[]

    ahrs.updateIMU(
        imu963r.gyro[0]*IMU_calibration,  imu963r.gyro[2]*IMU_calibration, -imu963r.gyro[1]*IMU_calibration,
        imu963r.acc[0],   imu963r.acc[2],  -imu963r.acc[1]
    );
}
```

### 建议修复方向

1. **延长校准采样** — 将 `calibrate_offsets(100)` 改为 `calibrate_offsets(500)`，采集 1 秒数据
2. **增大 Madgwick β** — 将 `MadgwickAHRS ahrs(100)` 改为 `MadgwickAHRS ahrs(100, 0.5f)`，增强加速度计修正
3. **增强低通滤波** — 将 `IMUHandler(float a_acc = 0.1f, float a_gyro = 0.5f)` 中 `a_gyro` 默认值减小到 `0.1f`
4. **添加死区** — 对低于阈值的陀螺值置零，抑制微小噪声
5. **Yaw 漂移** — 若无磁力计（IMU660RA/B），Yaw 漂移不可避免，需外接磁力计或使用其他航向参考

---

## 四、功能：摄像头预览挂载到菜单

### 需求

将 `my_task_function.cpp` 中的 `camera_test` 函数挂载到菜单系统，实现实时摄像头预览。

### 修改

#### 1. `my_menu.hpp:113` — 添加函数声明

```cpp
static void camera_test(uint8 cl_action);
```

#### 2. `my_menu.cpp:28` — 添加菜单条目

```cpp
{ 12,0,"camera_test",MyMenu::camera_test}
```

挂载在 Main Menu 根层级（`parent_id=0`），与 `motor_test` 同级。

#### 3. `my_menu.cpp:681-722` — 实现摄像头实时预览函数

```cpp
void MyMenu::camera_test(uint8 cl_action)
{
    static bool camera_inited = false;

    // 首次进入时初始化摄像头（car_init 被注释，需在此懒加载）
    if (!camera_inited) {
        printf("------------初始化摄像头------------\n");
        uvc.init("/dev/video0");
        camera_inited = true;
    }

    // 按键处理
    switch (cl_action) {
    case 3: // BACK → 返回菜单
        g_menu_instance->mode_inter_flag = 0;
        return;
    default: break;  // WAITING/OK 等都执行刷新
    }

    // 先获取上一帧的灰度图，再等待新帧
    gray_img_ptr = uvc.get_gray_image_ptr();
    int refresh_ret = uvc.wait_image_refresh();  // 0=成功, -1=失败

    zf_device_ips200* disp = g_menu_instance->ips_display;
    disp->clear();
    disp->print(0, 0, RGB565_WHITE, 20.0f, "=== Camera Test ===");

    if (refresh_ret == 0 && gray_img_ptr != nullptr) {
        disp->show_gray_image(0, 24, gray_img_ptr, UVC_WIDTH, UVC_HEIGHT);
    } else {
        disp->print(0, 100, "No camera frame...");
    }

    disp->draw_line(0, 290, disp->get_width(), 290, RGB565_BLACK);
    disp->print(0, 294, "[BACK] Return");
    disp->update();
}
```

### 设计要点

- **懒初始化**：`static bool` 标志确保摄像头只在首次进入时初始化一次
- **实时刷新**：利用之前修复的交互模式空闲刷新机制，无按键时也持续更新画面
- **BACK 退出**：按返回键回到菜单，摄像头不关闭再进入立即可用

### 最终菜单结构

```
Main Menu
├── mode1
│   ├── key_remap_test
│   └── imu_angle_display
├── mode2
│   └── brushless_calibration
├── mode3 ~ mode7
├── motor_test
└── camera_test        ← 新增
```

---

## 五、问题四：摄像头显示 "No camera frame"

### 现象

终端日志显示摄像头初始化成功（型号、分辨率、帧率均正常），但 IPS 屏幕显示 "No camera frame..."。

### 终端日志

```
------------初始化摄像头------------
find uvc camera Successfully.
等待摄像头就绪...
摄像头真正就绪，耗时 0 ms
get uvc width = 320
get uvc height = 240
get uvc fps = 60
ips屏幕显示no camera frame
```

### 根因

`wait_image_refresh()` 的返回值语义是 **`0`=成功，`-1`=失败**（见 `zf_device_uvc.cpp:112-140`），但条件判断用了 C 风格布尔：

```cpp
int refresh_ret = uvc.wait_image_refresh();  // 成功返回 0
if (refresh_ret && gray_img_ptr != nullptr)  // 0 是 falsy → 永远进不来！
```

| 返回值 | 含义 | `if (refresh_ret)` |
|--------|------|---------------------|
| `0` | **成功** | **falsy → else 分支❌** |
| `-1` | 失败 | truthy → if 分支 |

所以每次 `wait_image_refresh()` 成功返回 `0`，反而走了 `else` 打印 "No camera frame..."。

### 修复

**文件**：`project/code/my_menu.cpp`

两处改动：

1. 调换顺序：先 `get_gray_image_ptr()` 取上一帧灰度，再 `wait_image_refresh()` 抓新帧（与原版 `camera_test()` 一致）
2. 修正条件：`refresh_ret == 0` 替代 `refresh_ret`

```cpp
// 修改前：
int refresh_ret = uvc.wait_image_refresh();
gray_img_ptr = uvc.get_gray_image_ptr();
if (refresh_ret && gray_img_ptr != nullptr) {

// 修改后：
gray_img_ptr = uvc.get_gray_image_ptr();
int refresh_ret = uvc.wait_image_refresh();  // 0=成功, -1=失败
if (refresh_ret == 0 && gray_img_ptr != nullptr) {
```

---

## 六、涉及文件清单

| 文件 | 作用 | 修改 |
|------|------|------|
| `project/user/main.cpp` | 主入口，初始化流程 | ✅ 添加 `imu963r.init()` |
| `project/code/my_menu.cpp` | 菜单系统实现 | ✅ 空闲刷新 + camera_test |
| `project/code/my_menu.hpp` | 菜单系统头文件 | ✅ 声明 camera_test |
| `project/code/my_global.cpp` | 全局对象，线程回调 | 未修改（需关注漂移） |
| `project/code/my_global.hpp` | 全局对象声明 | 未修改 |
| `project/code/IMU963R.cpp` | IMU 数据处理，滤波，校准 | 未修改（需关注漂移） |
| `project/code/IMU963R.hpp` | IMU 数据处理头文件 | 未修改 |
| `project/code/MadgwickAHRS.cpp` | Madgwick 姿态解算 | 未修改 |
| `project/code/MadgwickAHRS.hpp` | Madgwick 姿态解算头文件 | 未修改 |
| `libraries/zf_device/zf_device_imu.cpp` | 底层 IIO sysfs 驱动 | 未修改 |
| `libraries/zf_device/zf_device_imu.hpp` | IIO 路径宏 + 类声明 | 未修改 |
| `libraries/zf_device/zf_device_uvc.cpp` | UVC 摄像头驱动 | 未修改 |
| `libraries/zf_device/zf_device_uvc.hpp` | UVC 摄像头头文件 | 未修改 |

### IIO 设备路径

定义在 `libraries/zf_device/zf_device_imu.hpp:44-54`：

```cpp
#define IMU_EVENT_PATH  "/sys/bus/iio/devices/iio:device1/events/in_voltage_change_en"
#define IMU_ACC_X_PATH  "/sys/bus/iio/devices/iio:device1/in_accel_x_raw"
#define IMU_ACC_Y_PATH  "/sys/bus/iio/devices/iio:device1/in_accel_y_raw"
#define IMU_ACC_Z_PATH  "/sys/bus/iio/devices/iio:device1/in_accel_z_raw"
#define IMU_GYRO_X_PATH "/sys/bus/iio/devices/iio:device1/in_anglvel_x_raw"
#define IMU_GYRO_Y_PATH "/sys/bus/iio/devices/iio:device1/in_anglvel_y_raw"
#define IMU_GYRO_Z_PATH "/sys/bus/iio/devices/iio:device1/in_anglvel_z_raw"
#define IMU_MAG_X_PATH  "/sys/bus/iio/devices/iio:device1/in_magn_x_raw"
#define IMU_MAG_Y_PATH  "/sys/bus/iio/devices/iio:device1/in_magn_y_raw"
#define IMU_MAG_Z_PATH  "/sys/bus/iio/devices/iio:device1/in_magn_z_raw"
```

> ⚠️ 如果板子上实际 IIO 设备编号不同（如 `iio:device0`），需修改全部 10 个宏。
