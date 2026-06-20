/*********************************************************************************************************************
* 文件名称          niu_vision.hpp
* 功能说明          牛爷爷视觉运动方案 — 头文件（全函数声明 + 全局变量 + 宏定义）
* 适用平台          LS2K0300
* 来源             牛爷爷视觉运动代码 (CYH-NYY)
* 移植日期          2026-06-19
********************************************************************************************************************/

#ifndef __NIU_VISION_HPP__
#define __NIU_VISION_HPP__

#include "zf_common_typedef.hpp"
#include <math.h>

// ======================================================================
// 图像参数（MT9V03X 摄像头）
// ======================================================================
#define MT9V03X_W  188
#define MT9V03X_H  120

// ======================================================================
// 八邻域方向数组（逆时针：上、右上、右、右下、下、左下、左、左上）
// ======================================================================
extern const int guize[8][2];

// ======================================================================
// 边线数据结构（原始 → 逆透视 → 滤波 → 重采样）
// ======================================================================
extern float L_edge[100][2];      // 左边界（八邻域输出 → touxian 原地改写）
extern float Lout_edge[100][2];   // 左边界滤波后
extern float Luse_edge[100][2];   // 左边界重采样后
extern float R_edge[100][2];      // 右边界
extern float Rout_edge[100][2];   // 右边界滤波后
extern float Ruse_edge[100][2];   // 右边界重采样后
extern int   L_count;             // 左边界有效点数
extern int   R_count;             // 右边界有效点数
extern int   Lout_count;          // 左边界滤波后点数
extern int   Rout_count;          // 右边界滤波后点数

extern float Lin_MID[50][2];      // 左中线
extern float Lout_MID[50][2];     // 左中线滤波后
extern float Rin_MID[50][2];      // 右中线
extern float Rout_MID[50][2];     // 右中线滤波后
extern int   Lmidnum;             // 左中线点数
extern int   Lmidnum2;            // 左中线滤波后点数
extern int   Rmidnum;             // 右中线点数
extern int   Rmidnum2;            // 右中线滤波后点数

// ======================================================================
// 最长白列（八邻域爬线起点）
// ======================================================================
extern int lwline;                // 最长白列所在列号
extern int lw;                    // 该列白点最上方行号

// ======================================================================
// 二值化图像（需由调用方填充）
// ======================================================================
extern uint8 image_use[MT9V03X_H][MT9V03X_W]; // 二值化后图像
extern uint8 image_see[MT9V03X_H][MT9V03X_W]; // 逆透视显示图像
extern uint8 show[MT9V03X_H][MT9V03X_W];      // 显示缓冲区
extern uint8 daj;                              // 大津法阈值

// ======================================================================
// 拐点检测变量
// ======================================================================
extern float ang_L[100];         // 左边界角度变化率
extern float ang_R[100];         // 右边界角度变化率
extern int   ang_numL;           // 左拐点索引
extern int   ang_numR;           // 右拐点索引
extern int   ifzhi_L;            // 左边界直道标志（1=直道）
extern int   ifzhi_R;            // 右边界直道标志
extern float conmax;             // 最小二乘残差（直道判定）

// ======================================================================
// 巡线策略状态
// ======================================================================
extern int xunxian_dir;          // 巡线方向 -1=左 1=右

// ======================================================================
// 圆环状态
// ======================================================================
extern int yuanhuan;             // 0=无 1=入环 2=环中 3=出环
extern int dir_yuanhuan;         // 圆环方向 1=右环 -1=左环
extern int suo;                  // 状态锁
extern int yanshi;               // 延时计数
extern int juli;                 // 距离积分

// ======================================================================
// 十字状态
// ======================================================================
extern int shizi;                // 十字标志
extern int len_shizi;            // 十字距离积分

// ======================================================================
// 斑马线/气泡
// ======================================================================
extern int banmacishu;           // 斑马线计数
extern int qipao;                // 气泡标志
extern int qipaojuli;            // 气泡距离

// ======================================================================
// 路障
// ======================================================================
extern int luzhang;              // 路障标志
extern int luzhangjuli;          // 路障距离
extern int dir_luzhang;          // 路障方向

// ======================================================================
// 坡道
// ======================================================================
extern int podao;                // 坡道标志 0=无 1=上坡 2=坡顶
extern int pochang;              // 坡道积分
extern int poyanshi;             // 坡道延时

// ======================================================================
// 减速
// ======================================================================
extern int jian;                 // 减速标志
extern int jianms;               // 减速延时

// ======================================================================
// 控制变量
// ======================================================================
extern float error;              // 当前偏差角度（度）
extern float error_last;         // 上一帧偏差
extern float duty;               // 舵机占空比
extern float speedtobel;         // 左轮期望速度
extern float speedtober;         // 右轮期望速度
extern float speedl;             // 左轮实际速度
extern float speedr;             // 右轮实际速度
extern float pwml;               // 左轮 PWM
extern float pwmr;               // 右轮 PWM
extern float P_fx;               // 方向 P 参数
extern float D_fx;               // 方向 D 参数
extern int   yumao;              // 预瞄距离（点数）
extern float rood_wid;           // 赛道宽度（像素）

// ======================================================================
// 逆透视矩阵（TOOD: 用 MATLAB 逆透视工具重新标定后替换）
//
// 标定流程：
//   1. 用小车摄像头拍摄正前方直道照片
//   2. 运行 视觉/逆透视/逆透视/逆透视.exe（或 MATLAB 脚本）
//   3. 在图像上标定赛道矩形的 4 个角点
//   4. 将 fprintf 打印出的 rot/inv_rot 替换下方矩阵
//
// 当前矩阵为 MATLAB 预设坐标的近似值（仅供参考，必须重标定）
// ======================================================================
extern const float rot[3][3];
extern const float inv_rot[3][3];

// ======================================================================
// 逆透视辅助函数
// ======================================================================
float Xrot_point(float x, float y);
float Yrot_point(float x, float y);
int   inv_rot_y(float x, float y);
float XShowRotimg(float x, float y);
float YShowRotimg(float x, float y);

// ======================================================================
// 工具函数（内联）
// ======================================================================
inline int niu_clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

inline float niu_fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}

// ======================================================================
// 函数声明
// ======================================================================

// --- 图像预处理 ---
uint8 otsuThreshold(uint8 *image);

// --- 边线搜索 ---
void   get_highest(void);
uint8  cbh(int x, int y, int k);
void   get_BLY(float L_line[][2], int *Lnum, float R_line[][2], int *Rnum, int len);

// --- 边线后处理 ---
void   blur_points(float img_in[][2], float imag_out[][2], int num, int kernel);
void   resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);

// --- 中线生成 ---
void   get_mid_L(float line[][2], int num, float mid[][2], int *num2, int approx_num);
void   get_mid_R(float line[][2], int num, float mid[][2], int *num2, int approx_num);

// --- 拐点检测 ---
void   local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void   nms_angle(float angle_in[], int num, float angle_out[], int kernel);
void   guai_mum(float Edge[][2], int num, float ang_out[], int *posi, int *if_zhi);

// --- 逆透视 ---
void   touxian(float line[][2], int *num);
void   nitoushi(void);

// --- 控制 ---
void   get_error(void);
void   madasudu(void);

// --- 主调度 ---
void   search_bianxian(void);
void   xunxain_celue(void);

// --- 桥接函数（连接摄像头 → 牛爷爷管线） ---
void   niu_vision_pipeline(uint8 *gray_ptr, int img_w, int img_h);

// --- 元素处理 ---
void   if_yuanhuan(void);
void   if_shizi(void);
void   if_luzhang(void);
void   if_po(void);
void   if_banmaxian(void);
void   jiansu(void);

#endif // __NIU_VISION_HPP__