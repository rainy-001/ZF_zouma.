# 扫线调试功能开发记录

> 日期：2026-06-12
> 目的：在 IPS 屏幕上实时显示二值图像 + 边线叠加 + 调试参数，通过按键动态调参

---

## 改动文件清单

| 文件 | 改动 |
|------|------|
| `project/code/imgproc.hpp` | 添加 `adapt_clip`、`tracking_decision_machine`、`cricle_decision_machine` 的 extern 声明 |
| `project/code/my_menu.hpp` | 添加 `line_scan_debug()` handler 声明 |
| `project/code/my_menu.cpp` | 菜单表新增 `line_scan` 条目 + 实现 `line_scan_debug()` 函数 (~170行) |

---

## 使用方式

1. 编译部署后，在 Main Menu 选择 **"line_scan"** 进入扫线调试模式
2. 摄像头会懒初始化（首次进入时自动 `uvc.init("/dev/video0")`）
3. 每帧自动执行 `wait_image_refresh()` → `image_proc()` → 显示二值图+边线

### IPS 屏幕布局

```
┌──────────────────────────────────┐
│                                  │
│    二值图像 (160×120)             │
│    · 白色 = 赛道 (> start_thre)  │
│    · 黑色 = 线/背景               │
│    · 红色线 = 左边界(Lline)       │
│    · 蓝色线 = 右边界(Rline)       │
│    · 黄色十字 = 角点位置          │
│                                  │
├──────────────────────────────────┤
│ >> start_thre: 130    ←当前调整项│
│ L:45 R:42 M:38  st:0/0  边线/状态│
│ nmsL:85@12  nmsR:92@14   角点值  │
│ onto:0.52  maxA:98.1  avgA:12.3  │
│ [OK]切换  [UP/DOWN]调参  [BACK]  │
└──────────────────────────────────┘
```

### 按键操作

| 按键 | 功能 |
|------|------|
| **OK** | 循环切换调整参数 (start_thre → all_block_size → adapt_clip) |
| **UP** | 当前选中参数 + (start_thre +5, all_block_size +2, adapt_clip +1) |
| **DOWN** | 当前选中参数 - |
| **BACK** | 退出扫线调试，返回主菜单 |

### 可调参数

| 参数 | 作用 | 默认值 | 范围 |
|------|------|--------|------|
| `start_thre` | Otsu 二值化阈值（控制黑白分割敏感度） | Otsu 自动计算 | 10~255 |
| `all_block_size` | 迷宫巡线自适应阈值块大小（越大越平滑但细节丢失） | 7 | 3~25 |
| `adapt_clip` | 自适应阈值偏差量（控制边界敏感度） | 7 | 1~50 |

---

## 实现细节

### 二值图像生成
```cpp
// 用 Otsu 阈值对 img_gray 逐像素二值化到静态缓冲区 bin_buf[120][160]
bin_buf[i][j] = (img_gray[i * IMG_W + j] > start_thre) ? 255 : 0;
```
- 白(255) = 赛道区域
- 黑(0) = 线/背景

### 边线叠加
- 遍历 `Lline[]` 用红色 `draw_line` 连接相邻点（左边界）
- 遍历 `Rline[]` 用蓝色 `draw_line` 连接相邻点（右边界）
- 在 `nms_Lline_idx` / `nms_Rline_idx` 处绘制黄色十字标记角点

### 依赖
- `image_proc()` 需要先调用 `uvc.wait_image_refresh()` 获取帧
- 状态机变量 `tracking_decision_machine` / `cricle_decision_machine` 已通过新增 extern 导出
- `adapt_clip` 原来缺少 extern 声明，已补充

### 已知限制
- 角点索引 (`nms_Lline_idx`) 是 `sampled_Lline` 的索引（透视变换后），但在显示时直接用于 `Lline` 坐标（原始图像坐标），因为透视变换后的索引无法直接映射回原图。影响：角点标记位置可能有少许偏移
- 中线 (`Mline`) 未绘制，因为中线在透视/世界坐标系中，需要逆变换回图像坐标
