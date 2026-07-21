#ifndef _IMGPROC_HPP__
#define _IMGPROC_HPP_

#include "zf_common_typedef.hpp"
#include "zf_device_uvc.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdint>
#include <vector>
#include "my_timestamp.hpp"

// ========================================== 图像尺寸 (适配 160x120) ==========================================
#define IPSH 120                             // 图像处理高度 (原 60 → 120)
#define IPSW 160                             // 图像处理宽度 (原 80 → 160)
#define IMG_W  IPSW                          // 向后兼容 zouma 旧代码
#define IMG_H  IPSH                          // 向后兼容 zouma 旧代码
#define PictureCentring 80                   // 图像中心列 (原 40 → 80)

// ========================================== 边界限幅宏 ==========================================
#define LimitL(L) (L = ((L < 2) ? 2 : L))    // 左边界限幅 (原 1 → 2)
#define LimitH(H) (H = ((H > 157) ? 157 : H))// 右边界限幅 (原 78 → 157)

// ========================================== 车道元素类型枚举 ==========================================
typedef enum {
    Straight,       // 直道
    Ramp,           // 坡道
    LeftCirque,     // 左圆环
    RightCirque,    // 右圆环
    Cross_ture,     // 十字
    Barn_in,        // 入库
} RoadType_e;

// ========================================== 图像状态结构 ==========================================
typedef struct {
    RoadType_e Road_type;       // 元素类型
    int16_t Left_Line;          // 左线丢边数
    int16_t WhiteLine;          // 双边丢边数
    int16_t Right_Line;         // 右线丢边数
} ImageStatustypedef;

// ========================================== 图像处理参数结构 ==========================================
typedef struct {
    uint8_t Threshold;                     // 二值化阈值
    uint32_t Threshold_static;             // 二值化静态下限
    uint8_t Threshold_detach;              // 大津法搜索范围上限
    RoadType_e Road_type;                  // 元素类型
    uint8_t OFFLine;                       // 图像截止行
    uint8_t WhiteLine;                     // 双边丢边计数
    int16_t Miss_Left_lines;               // 左线丢失计数
    int16_t Miss_Right_lines;              // 右线丢失计数
    int TowPoint;                          // 控制误差的前瞻
    int Det_True;                          // 检测到的偏差 (像素列偏移)
    int16_t Zebra_Flag;                    // 斑马线标志
    int16_t Ramp_Flag;                     // 坡道标志
    float MU_P;
    int straight_acc;                      // 直道加速标志
    int variance_acc;                      // 方差阈值
    int16_t OFFLineBoundary;               // 八邻域截止行
    // 左右手法则扫线数据
    int16_t WhiteLine_L;                   // 左边丢线数
    int16_t WhiteLine_R;                   // 右边丢线数
    int16_t image_element_rings;           // 0:无圆环 1:左圆环 2:右圆环
    int16_t image_element_rings_flag;      // 圆环状态机 flag
    int16_t ring_big_small;                // 0:无 1:大圆环 2:小圆环
    uint32_t ring_state_counter;           // 圆环状态计数器(超时保护)
    uint16_t ring_state_last_flag;         // 上次圆环 flag (检测状态变化)
} ImageParametertypedef;

// ========================================== 跳变点结构 ==========================================
typedef struct {
    int point;
    uint8_t type;  // 'T'=正常跳变, 'W'=无边行, 'H'=大跳变
} JumpPointtypedef;

// ========================================== 行属性结构 (每行边界信息) ==========================================
typedef struct {
    uint8_t IsRightFind;       // 右边有边标志
    uint8_t IsLeftFind;        // 左边有边标志
    int Wide;                  // 边界宽度
    int LeftBorder;            // 左边界
    int RightBorder;           // 右边界
    int Center;                // 中线
    int LeftBoundary_First;    // 左边界(第一次经过)
    int RightBoundary_First;   // 右边界(第一次经过)
    int LeftBoundary;          // 左边界(最后一次经过)
    int RightBoundary;         // 右边界(最后一次经过)
} RowAttributetypedef;

// ========================================== 速度数据结构 ==========================================
typedef struct {
    float nowspeed;
    int MinSpeed;
    int MaxSpeed;
    int straight_speed;
} SpeedDatatypedef;

// ========================================== 系统数据结构 ==========================================
typedef struct {
    SpeedDatatypedef SpeedData;
    uint8_t Stop;
    int exp_time;
    int mtv_lroffset;
    int mtv_gain;
} SystemDatatypdef;

// ========================================== 外部变量声明 ==========================================
// --- 716 移植过来的全局变量 ---
extern ImageStatustypedef ImageStatus;
extern RowAttributetypedef RowAttribute[IPSH];
extern ImageParametertypedef ImageParameter;
extern SystemDatatypdef SystemData;

extern uint8_t Cramp_image[IPSH][IPSW];     // 压缩后的灰度图像
extern uint8_t heheImage[IPSH][IPSW];       // 二值化后的图像
extern uint8_t color_image[IPSH][IPSW][3];  // 彩色图像 (BGR三通道)

// 红色物块检测相关变量
extern int red_block_detected;
extern int red_block_center_x;
extern int red_block_center_y;
extern int extract_region_x;
extern int extract_region_y;
extern int extract_region_size;

extern uint32_t Cross_TPoint;
extern uint32_t Rings_TPoint;
extern uint32_t Straight_TPoint;
extern uint32_t encoder_distance, ramp_distance, stop_distance;
extern float variance, variance_acc;

extern int ImageScanInterval;
extern int ImageScanInterval_Cross;
extern int Det_True;

// --- zouma 原有输出接口 (保持兼容) ---
extern float onto;                  // 最终处理方向，已限制幅度在-30~30.0f之间
extern float angle_compensation;    // 方向补偿量(静态最中间偏差)

// --- zouma 原有图像变量 (保持兼容) ---
extern cv::Mat frame_color;
extern cv::Mat frame_gray;
extern cv::Mat frame_bin;
extern uint8_t* img_gray;
extern uint8_t bin_img_data[IPSW * IPSH];

// UVC 摄像头对象 (在 my_global.cpp 中定义)
extern zf_device_uvc uvc;

// ========================================== 函数声明 ==========================================
// 图像预处理
void Cramping(void);
void Thershold_separation_Otsu(void);
void Bin_Image_Filter(void);

// 边线追逐扫线
void JumpPointAndType(uint8_t* p, uint8_t type, int L, int H, JumpPointtypedef* Q);
void Search_Bottom_Line_OTSU(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline);
void Search_Left_and_Right_Lines(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline);
void Search_Border_OTSU(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline);
void Get_ExtensionLine(void);
void DrawExtensionLine(void);

// 中线与滤波
void RouteFilter(void);

// 前瞻与误差
void Prospective_error(void);
void Update_Dynamic_Lookahead(void);
void Speed_Control_Factor(void);

// 直道检测
void Straightacc_Test(void);
float Straight_Judge(uint8_t dir, uint8_t start, uint8_t end);

// 元素检测
void Element_Judgment_Left_Rings(void);
void Element_Judgment_Right_Rings(void);
void Element_Judgment_Ramp(void);
void Element_Judgment_Zebra(void);
void Element_Test(void);

// 元素处理
void Element_Handle_Left_Rings(void);
void Element_Handle_Right_Rings(void);
void Element_Handle_Small_Rings(void);
void Element_Handle_Big_Rings(void);
void Element_Handle(void);
void element_I(void);

// 十字处理
int Find_Down_Point(int start, int end);

// 红色物块识别
int detect_red_color_block(void);
int detect_red_block_classify(void);

// 出界保护
void Emergency_Breaking(void);

// 总处理函数
void ImageProcess(void);
void image_proc(void);   // zouma 原有接口

// 调试
void PrintBorders(const RowAttributetypedef* rowAttributes, int numRows);

// 圆环保护
void Check_Ring_State_Timeout(void);

// 获取时间 (在 my_global.cpp/config_setting.cpp 中已有)
uint32_t Get_Time_Ms(void);

#endif // _IMGPROC_HPP__
