# IMU + 视觉 融合方案

> 当前状态：视觉 (`onto`) 与 IMU 航向 (`calculate_yaw_control`) **各自独立，开关切换，未融合**。
> 三个融合层级由浅入深，逐步推进。

---

## 方案一：松耦合 — IMU Yaw 补偿视觉丢线

### 场景

摄像头因强光、阴影、弯道过大等原因丢线（边线点数 < `LOST_LINE` 阈值）时，视觉输出的 `onto` 不可信。

### 策略

丢线时**无缝切换**到 IMU 航向保持，恢复后切回视觉巡线。

### 伪代码

```cpp
// pid_contol_handle() 中

if (onto_pd_control_enable == 1) {
    bool visual_lost = (middle_line_length < LOST_LINE);  // 视觉丢线判断

    if (visual_lost) {
        // 丢线：锁定当前 Yaw 为目标，IMU 航向保持
        static float hold_yaw = 0;
        static bool  first_lost = true;

        if (first_lost) {
            hold_yaw   = ahrs.getYaw();   // 记录丢线瞬间的航向
            first_lost = false;
        }
        onto_control = pid_angle.compute(
            calculate_yaw_control(hold_yaw, ahrs.getYaw(), 25), 0);
    } else {
        first_lost   = true;              // 复位标志
        onto_control = pid_angle.compute(onto, 0.0f);  // 正常视觉巡线
    }
}
```

### 改动范围

| 文件 | 改动 |
|------|------|
| `my_global.cpp` | `pid_contol_handle()` 中增加丢线检测 + Yaw 保持分支 |

### 优点

- 改动最小，只改控制层，图像处理代码不动
- 能防止丢线瞬间车辆"盲开"失控

### 缺点

- 航向保持期间无法应对弯道（Yaw 不变 = 走直线或切线）
- 本质是"切换"而非"融合"

---

## 方案二：紧耦合 — IMU 辅助视觉元素识别

### 场景

视觉检测十字路口、圆环（环岛）等赛道元素时，单帧判断容易误检。IMU 的 Yaw 角变化可作为物理约束进行交叉验证。

### 策略

利用 `Circle_Tracking_Machine_TypeDef` 中已定义但未使用的 `start_angle`/`current_angle` 字段，在进入元素时记录 IMU 航向，出元素时校验角度变化是否符合预期。

### 圆环（环岛）验证示例

```cpp
// 在 circle_process() 或 element_status() 中

void circle_process() {
    switch (cricle_decision_machine.state) {
    case 1:  // 进入圆环路口
        cricle_decision_machine.start_angle = ahrs.getYaw();  // 记录入环航向
        break;

    case 2:  // 环岛中
        cricle_decision_machine.current_angle = ahrs.getYaw();
        // 实时监控角度变化，辅助判断是否仍在绕行
        break;

    case 4: {  // 出环岛路口
        float delta_yaw = ahrs.getYaw() - cricle_decision_machine.start_angle;
        // 圆环绕行一周，Yaw 变化应接近 ±360°
        if (fabs(delta_yaw) < 180.0f || fabs(delta_yaw) > 540.0f) {
            // 角度变化不合理，可能是视觉误检
            // 回退或重新确认
        }
        break;
    }
    }
}
```

### 十字路口验证示例

```cpp
// 十字路口是直线穿过，Yaw 不应剧烈变化
// 如果视觉判为十字但 Yaw 在剧烈变化 → 可能是急弯误判
float yaw_rate = fabs(ahrs.getYaw() - prev_yaw) / dt_ms * 1000;  // deg/s
if (yaw_rate > 300.0f && state == STATE_CROSSING) {
    // 航向变化太快，十字误判，纠正为弯道
    state = STATE_CURVE;
}
```

### 改动范围

| 文件 | 改动 |
|------|------|
| `imgproc.cpp` | `circle_process()`、`crossing_process()` 中加入 IMU 校验 |
| `imgproc.hpp` | `Circle_Tracking_Machine` 字段已存在，无需改动 |

### 优点

- 利用 IMU 的物理约束减少视觉误判
- 字段和结构体已预留，改动量小
- 不改变核心巡线逻辑

### 缺点

- IMU Yaw 有漂移（无磁力计），长时间绕行累积误差可能影响判断
- 需要调参确定合理的角度阈值

---

## 方案三：全状态估计 — EKF 融合位姿估计

### 场景

将视觉检测的赛道边界、IMU 姿态、编码器里程全部纳入一个**扩展卡尔曼滤波器 (EKF)**，输出最优车辆位姿估计 `(x, y, θ)`。

### 架构

```
┌──────────┐  ┌───────────┐  ┌───────────┐
│ 编码器    │  │ IMU       │  │ 摄像头     │
│ (里程)   │  │ (角速度)   │  │ (边界点)  │
└────┬─────┘  └─────┬─────┘  └─────┬─────┘
     │              │              │
     ▼              ▼              ▼
┌─────────────────────────────────────────┐
│            EKF (扩展卡尔曼滤波器)         │
│                                         │
│  预测步骤:                               │
│    δs = (δs_L + δs_R) / 2               │
│    θ_k = θ_{k-1} + ω_z * Δt             │
│    x_k = x_{k-1} + δs * cos(θ_k)        │
│    y_k = y_{k-1} + δs * sin(θ_k)        │
│                                         │
│  更新步骤:                               │
│    视觉观测 → 赛道中线点到车体的偏移     │
│    → 校正 (x, y, θ) 的估计              │
│                                         │
│  输出: 最优位姿 (x̂, ŷ, θ̂)               │
└────────────────────┬────────────────────┘
                     ▼
            ┌────────────────┐
            │  MPC / 纯追踪   │
            │  转向控制       │
            └────────────────┘
```

### 状态向量与运动模型

```
状态: X = [x, y, θ]^T                    (2D 位置 + 航向)

预测 (运动模型):
    θ_k = θ_{k-1} + ω_z * Δt + ε_θ
    x_k = x_{k-1} + v * cos(θ_k) * Δt + ε_x
    y_k = y_{k-1} + v * sin(θ_k) * Δt + ε_y

其中:
    ω_z = gyro_z  (IMU 角速度)
    v    = (v_L + v_R) / 2  (编码器线速度)
```

### 观测模型

```
视觉观测 z = [d, φ]^T

    d = 车体到赛道中线的横向偏移    (从透视变换后的 Mline 计算)
    φ = 赛道中线的切线方向           (从 Mline 局部角度计算)

观测方程:
    z_k = h(X_k) + η
    = [ cross_track_error(x_k, y_k, θ_k, Mline),
        tangent_angle(x_k, y_k, θ_k, Mline) ]^T + η
```

### 核心代码框架

```cpp
// ekf_fusion.hpp

struct EKF {
    // 状态向量
    float x, y, theta;           // 位姿

    // 协方差矩阵 (3x3)
    float P[3][3];

    // 过程噪声 & 观测噪声
    float Q[3][3];               // 编码器+IMU 噪声
    float R[2][2];               // 视觉观测噪声

    // --- 预测步骤 (每 10ms, 与 IMU 同步) ---
    void predict(float gyro_z, float v, float dt) {
        // 1. 状态预测
        float theta_pred = theta + gyro_z * dt;
        float x_pred     = x + v * cos(theta_pred) * dt;
        float y_pred     = y + v * sin(theta_pred) * dt;

        // 2. 计算雅可比 F (状态转移矩阵)
        float F[3][3] = {
            {1, 0, -v * sin(theta_pred) * dt},
            {0, 1,  v * cos(theta_pred) * dt},
            {0, 0,  1}
        };

        // 3. 协方差预测: P = F * P * F^T + Q
        mat_mul_3x3(F, P, P);
        mat_add(P, Q);

        x = x_pred; y = y_pred; theta = theta_pred;
    }

    // --- 更新步骤 (每帧图像到达时, ~50-100ms) ---
    void update_from_vision(float cross_track_err, float tangent_angle) {
        // 1. 计算观测残差 y_tilde = z - h(x)
        //    (需要根据 Mline 坐标和当前位姿计算预期观测)
        float z_pred[2];  predict_observation(x, y, theta, Mline);
        float y_tilde[2] = {cross_track_err - z_pred[0],
                            tangent_angle  - z_pred[1]};

        // 2. 观测雅可比 H (2x3)
        float H[2][3];   compute_H(x, y, theta);

        // 3. 卡尔曼增益: K = P * H^T * (H * P * H^T + R)^{-1}
        float S[2][2], K[3][2];
        // ... 矩阵运算 ...

        // 4. 状态更新: x = x + K * y_tilde
        x += K[0][0]*y_tilde[0] + K[0][1]*y_tilde[1];
        y += K[1][0]*y_tilde[0] + K[1][1]*y_tilde[1];
        theta += K[2][0]*y_tilde[0] + K[2][1]*y_tilde[1];

        // 5. 协方差更新: P = (I - K*H) * P
        // ...
    }
};
```

### 改动范围

| 文件 | 改动 |
|------|------|
| **新建** `project/code/ekf_fusion.hpp` | EKF 类实现 |
| **新建** `project/code/ekf_fusion.cpp` | 矩阵运算 + 观测模型 |
| `my_global.cpp` | `pid_contol_handle()` 改为调用 EKF 输出 |
| `imgproc.cpp` | 提取 Mline 作为观测输入（目前已有，需导出） |
| `navigation.hpp` | 可与 PathTracker 合并或替换 |

### 优点

- 真正的**融合**，不是切换——每个传感器互补
- IMU 提供高频动态响应（100Hz），视觉修正低频漂移（~10Hz）
- 输出连续、平滑的位姿估计，不怕单传感器失效
- 为后续 MPC 控制打基础

### 缺点

- 实现复杂度高，矩阵运算在嵌入式上需小心优化
- Yaw 无磁力计修正，长时间积分仍有漂移——视觉观测是唯一的绝对参考
- 需要精确标定摄像头外参（安装位置、角度、透视矩阵）
- 观测模型 `h(X)` 的设计直接影响融合效果

---

## 推荐推进路线

```
第一步 (立即可做)       第二步 (调车阶段)         第三步 (赛前优化)
┌──────────────┐      ┌──────────────┐        ┌──────────────┐
│ 方案一        │  →   │ 方案二        │   →   │ 方案三        │
│ IMU 丢线补偿  │      │ IMU 辅助元素  │        │ EKF 全融合    │
│              │      │ 交叉验证      │        │ 位姿估计      │
│ 改动: 20 行  │      │ 改动: 50 行   │        │ 新增: 300+ 行 │
│ 风险: 低     │      │ 风险: 低      │        │ 风险: 中高    │
└──────────────┘      └──────────────┘        └──────────────┘
```

方案一和二可以同时推进——方案一解决"丢线失控"的紧急问题，方案二提高元素识别的准确率。方案三在赛道稳定跑通后作为性能提升手段引入。
