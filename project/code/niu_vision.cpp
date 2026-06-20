/*********************************************************************************************************************
* 文件名称          niu_vision.cpp
* 功能说明          牛爷爷视觉运动方案 — 实现文件（全部函数体）
* 适用平台          LS2K0300
* 来源             牛爷爷视觉运动代码 (CYH-NYY)
* 移植日期          2026-06-19
********************************************************************************************************************/

#include "niu_vision.hpp"

// ======================================================================
// 逆透视矩阵（TODO: 用 MATLAB 逆透视工具重新标定后替换）
//
// 标定流程：
//   1. 用小车摄像头拍摄正前方直道照片，保存为 .jpg
//   2. 运行 视觉/逆透视/逆透视/逆透视.exe（Windows 可执行文件）
//   3. 在图像上标定赛道矩形的 4 个角点，输入正方形中心 Y 值
//   4. 程序输出 rot[3][3] 和 inv_rot[3][3] 矩阵
//   5. 将输出替换下方对应矩阵，并更新本注释中的标定日期
//
// 当前矩阵 = 单位矩阵（无变换效果，调试通过后再替换）
// 标定日期：待标定
// ======================================================================
const float rot[3][3] = {
    { 1.0f,  0.0f,  0.0f },
    { 0.0f,  1.0f,  0.0f },
    { 0.0f,  0.0f,  1.0f }
};

const float inv_rot[3][3] = {
    { 1.0f,  0.0f,  0.0f },
    { 0.0f,  1.0f,  0.0f },
    { 0.0f,  0.0f,  1.0f }
};

// ---------------------------------------------------------------------
// 逆透视辅助函数（点映射）
// ---------------------------------------------------------------------
float Xrot_point(float x, float y) {
    float W = x * rot[2][0] + y * rot[2][1] + rot[2][2];
    if (W == 0.0f) return 0.0f;
    return (x * rot[0][0] + y * rot[0][1] + rot[0][2]) / W;
}

float Yrot_point(float x, float y) {
    float W = x * rot[2][0] + y * rot[2][1] + rot[2][2];
    if (W == 0.0f) return 0.0f;
    return (x * rot[1][0] + y * rot[1][1] + rot[1][2]) / W;
}

int inv_rot_y(float x, float y) {
    float W = x * inv_rot[2][0] + y * inv_rot[2][1] + inv_rot[2][2];
    if (W == 0.0f) return 0;
    float Y = (x * inv_rot[1][0] + y * inv_rot[1][1] + inv_rot[1][2]) / W;
    return (int)(Y + 0.5f);
}

float XShowRotimg(float x, float y) {
    float W = x * inv_rot[2][0] + y * inv_rot[2][1] + inv_rot[2][2];
    if (W == 0.0f) return 0.0f;
    return (x * inv_rot[0][0] + y * inv_rot[0][1] + inv_rot[0][2]) / W;
}

float YShowRotimg(float x, float y) {
    float W = x * inv_rot[2][0] + y * inv_rot[2][1] + inv_rot[2][2];
    if (W == 0.0f) return 0.0f;
    return (x * inv_rot[1][0] + y * inv_rot[1][1] + inv_rot[1][2]) / W;
}

// ======================================================================
// 静态内部变量：大津法阈值保护
// ======================================================================
static uint8 last_threshold = 128;

// ======================================================================
// 八邻域方向数组（逆时针：上、右上、右、右下、下、左下、左、左上）
// ======================================================================
const int guize[8][2] = {
    { 0, -1 },   // 0: 上
    { 1, -1 },   // 1: 右上
    { 1,  0 },   // 2: 右
    { 1,  1 },   // 3: 右下
    { 0,  1 },   // 4: 下
    {-1,  1 },   // 5: 左下
    {-1,  0 },   // 6: 左
    {-1, -1 }    // 7: 左上
};

// ======================================================================
// 全局变量定义
// ======================================================================

// --- 边线数组 ---
float L_edge[100][2];
float Lout_edge[100][2];
float Luse_edge[100][2];
float R_edge[100][2];
float Rout_edge[100][2];
float Ruse_edge[100][2];
int L_count = 0, R_count = 0;
int Lout_count = 0, Rout_count = 0;

// --- 中线数组 ---
float Lin_MID[50][2];
float Lout_MID[50][2];
float Rin_MID[50][2];
float Rout_MID[50][2];
int Lmidnum = 0, Lmidnum2 = 0;
int Rmidnum = 0, Rmidnum2 = 0;

// --- 最长白列 ---
int lwline = 94;
int lw = 119;

// --- 二值化图像 ---
uint8 image_use[MT9V03X_H][MT9V03X_W];
uint8 image_see[MT9V03X_H][MT9V03X_W];
uint8 show[MT9V03X_H][MT9V03X_W];
uint8 daj = 128;

// --- 拐点检测 ---
float ang_L[100];
float ang_R[100];
int ang_numL = 0;
int ang_numR = 0;
int ifzhi_L = 0;
int ifzhi_R = 0;
float conmax = 0.0f;

// --- 巡线策略 ---
int xunxian_dir = 0;

// --- 圆环 ---
int yuanhuan = 0, dir_yuanhuan = 0, suo = 0;
int yanshi = 0, juli = 0;

// --- 十字 ---
int shizi = 0, len_shizi = 0;

// --- 斑马线/气泡 ---
int banmacishu = 0;
int qipao = 0, qipaojuli = 0;

// --- 路障 ---
int luzhang = 0, luzhangjuli = 0, dir_luzhang = 0;

// --- 坡道 ---
int podao = 0, pochang = 0, poyanshi = 0;

// --- 减速 ---
int jian = 0, jianms = 0;

// --- 控制 ---
float error = 0.0f, error_last = 0.0f, duty = 0.0f;
float speedtobel = 0.0f, speedtober = 0.0f;
float speedl = 0.0f, speedr = 0.0f;
float pwml = 0.0f, pwmr = 0.0f;
float P_fx = 0.035f;   // 方向P参数（需实测调参）
float D_fx = 0.015f;   // 方向D参数
int   yumao = 12;      // 预瞄距离（需实测调参）
float rood_wid = 45.0f; // 赛道宽度（像素）

// ======================================================================
// 速度环 PID 参数（需实测调参）
// ======================================================================
static float KP_L = 45.0f, KI_L = 5.0f, KD_L = 0.0f;
static float KP_R = 45.0f, KI_R = 5.0f, KD_R = 0.0f;
static float L_err[3] = {0.0f, 0.0f, 0.0f};
static float R_err[3] = {0.0f, 0.0f, 0.0f};
static float L_sum = 0.0f, R_sum = 0.0f;
static float dl = 0.0f, dr = 0.0f;  // 上一帧差值
#define PID_I_LIMIT  4000   // 积分限幅
#define PWM_LIMIT    9000   // 占空比限幅

// ======================================================================
// 图像预处理 — 快速大津法
// ======================================================================
uint8 otsuThreshold(uint8 *image)
{
    int pixelCount[256] = {0};
    float pixelPro[256] = {0};
    int width  = MT9V03X_W;
    int height = MT9V03X_H - 1;   // 4;
    int Sumpix = width * height / 4;   // 隔2行2列采样
    uint8 threshold = 0;
    uint8 *data = image;

    // 统计灰度级中每个像素在整幅图像中的个数 —— 隔2行2列采样加速
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            pixelCount[(int)data[i * width + j]]++;
        }
    }

    float u = 0.0f;
    for (int i = 0; i < 256; i++) {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;
        u += i * pixelPro[i];
    }

    float maxVariance = 0.0f;
    float w0 = 0.0f, avgValue = 0.0f;
    for (int i = 0; i < 256; i++) {
        w0 += pixelPro[i];
        avgValue += i * pixelPro[i];

        float variance = (avgValue / w0 - u) * (avgValue / w0 - u) * w0 / (1.0f - w0);
        if (variance > maxVariance) {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }

    // 阈值保护：防止极端光照导致崩溃
    if (threshold > 40 && threshold < 200)
        last_threshold = threshold;
    else
        threshold = last_threshold;

    return threshold;
}

// ======================================================================
// 边线搜索 — 寻找最长白列
// ======================================================================
void get_highest(void)
{
    lwline = 94;
    lw = 119;

    for (int i = 64; i < 124; i += 3) {         // 隔三列扫描加速
        if (image_use[80][i] >= daj) {          // 80行必须为白
            for (int j = 85; j >= 0; j--) {
                // 找白→黑→黑跳变
                if ((image_use[j][i] >= daj && image_use[j-1][i] < daj && image_use[j-2][i] < daj) || j <= 5) {
                    if (j < lw) {   // 贪心取最短白列（最靠近图像顶部）
                        lw = j;
                        lwline = i;
                    }
                    break;
                }
            }
        }
    }
}

// ======================================================================
// 边线搜索 — 黑白判断（八邻域爬线核心）
// ======================================================================
static int hig = 18;  // 起始扫线行上移行数

uint8 cbh(int x, int y, int k)
{
    int i = x + guize[k][0];
    int j = y + guize[k][1];

    if (j >= MT9V03X_H - hig) return 1;           // 底部黑框
    else if (j <= 1 || i <= 3 || i >= MT9V03X_W - 4) return 2;  // 停止框
    else if (image_use[j][i] >= daj) return 0;     // 白点
    else return 1;                                  // 黑点
}

// ======================================================================
// 边线搜索 — 八邻域爬线（左右双线）
// ======================================================================
void get_BLY(float L_line[][2], int *Lnum, float R_line[][2], int *Rnum, int len)
{
    hig = len;
    int L_C = 0, R_C = 0;
    int L_amount = 100, R_amount = 100;

    // --- 爬左起始点 ---
    for (int i = (int)lwline; i > 0; i--) {
        if (cbh(i, MT9V03X_H - hig - 1, 6) == 1 &&
            cbh(i + 1, MT9V03X_H - hig - 1, 6) == 1 &&
            cbh(i + 2, MT9V03X_H - hig - 1, 6) == 0) {
            L_line[0][0] = (float)(i + 2);
            break;
        } else if (cbh(i + 1, MT9V03X_H - hig - 1, 6) == 2) {
            L_line[0][0] = (float)(i + 2);
            break;
        }
    }

    // --- 爬右起始点 ---
    for (int i = (int)lwline; i < MT9V03X_W; i++) {
        if (cbh(i, MT9V03X_H - hig - 1, 2) == 1 &&
            cbh(i - 1, MT9V03X_H - hig - 1, 2) == 1 &&
            cbh(i - 2, MT9V03X_H - hig - 1, 2) == 0) {
            R_line[0][0] = (float)(i - 2);
            break;
        } else if (cbh(i - 1, MT9V03X_H - hig - 1, 2) == 2) {
            R_line[0][0] = (float)(i - 2);
            break;
        }
    }

    L_line[0][1] = (float)(MT9V03X_H - hig - 1);
    R_line[0][1] = (float)(MT9V03X_H - hig - 1);
    L_C = 1;
    R_C = 1;

    int l_dirl = 6, l_dirr = 2;
    int dirl = 6, dirr = 2;

    // --- 追踪左边界 ---
    for (int i = 1; i < L_amount; i++) {
        int cishu = 0;

        // 停止条件：8方向任一碰到边界
        if (cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 0) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 1) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 2) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 3) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 4) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 5) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 6) == 2 ||
            cbh((int)L_line[i-1][0], (int)L_line[i-1][1], 7) == 2) break;

        for (int j = 0; j < 8; j++) {
            if (cbh((int)L_line[i-1][0], (int)L_line[i-1][1], (j + l_dirl + 4) % 8) == 1 &&
                cbh((int)L_line[i-1][0], (int)L_line[i-1][1], (j + l_dirl + 4 + 1) % 8) == 0 &&
                cbh((int)L_line[i-1][0], (int)L_line[i-1][1], (j + l_dirl + 4 + 2) % 8) == 0) {
                dirl = (j + l_dirl + 4 + 1) % 8;
                break;
            } else {
                cishu++;
            }
        }
        l_dirl = dirl;

        if (cishu == 8) break;

        L_line[i][0] = L_line[i-1][0] + (float)guize[dirl][0];
        L_line[i][1] = L_line[i-1][1] + (float)guize[dirl][1];
        L_C++;
    }

    // --- 追踪右边界 ---
    for (int i = 1; i < R_amount; i++) {
        int cishu = 0;

        if (cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 0) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 1) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 2) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 3) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 4) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 5) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 6) == 2 ||
            cbh((int)R_line[i-1][0], (int)R_line[i-1][1], 7) == 2) break;

        for (int j = 0; j < 8; j++) {
            if (cbh((int)R_line[i-1][0], (int)R_line[i-1][1], (8 - j + l_dirr + 4) % 8) == 1 &&
                cbh((int)R_line[i-1][0], (int)R_line[i-1][1], (8 - j + l_dirr + 4 - 1) % 8) == 0 &&
                cbh((int)R_line[i-1][0], (int)R_line[i-1][1], (8 - j + l_dirr + 4 - 2) % 8) == 0) {
                dirr = (8 - j + l_dirr + 4 - 1) % 8;
                break;
            } else {
                cishu++;
            }
        }
        l_dirr = dirr;

        if (cishu == 8) break;

        R_line[i][0] = R_line[i-1][0] + (float)guize[dirr][0];
        R_line[i][1] = R_line[i-1][1] + (float)guize[dirr][1];
        R_C++;
    }

    *Lnum = L_C;
    *Rnum = R_C;
}

// ======================================================================
// 边线后处理 — 三角滤波
// ======================================================================
void blur_points(float img_in[][2], float imag_out[][2], int num, int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        imag_out[i][0] = 0.0f;
        imag_out[i][1] = 0.0f;
        for (int j = -half; j <= half; j++) {
            int fu = i + j;
            if (fu < 0) fu = 0;
            if (fu > num - 1) fu = num - 1;
            imag_out[i][0] += img_in[fu][0] * (float)(half + 1 - abs(j));
            imag_out[i][1] += img_in[fu][1] * (float)(half + 1 - abs(j));
        }
        float weight_sum = (float)((2 * half + 2) * (half + 1) / 2);
        imag_out[i][0] /= weight_sum;
        imag_out[i][1] /= weight_sum;
    }
}

// ======================================================================
// 边线后处理 — 等距重采样（二分线性插值法）
// ======================================================================
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    if (num1 < 0) {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrtf(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrtf(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1.0f - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);
    }
    *num2 = len;
}

// ======================================================================
// 中线生成 — 左线推中线（向赛道内侧偏移半道宽）
// ======================================================================
void get_mid_L(float line[][2], int num, float mid[][2], int *num2, int approx_num)
{
    int mnum = 0;
    float dist = rood_wid / 2.0f;
    for (int i = 0; i < num; i++) {
        float dx = line[niu_clip(i + approx_num, 0, num - 1)][0] - line[niu_clip(i - approx_num, 0, num - 1)][0];
        float dy = line[niu_clip(i + approx_num, 0, num - 1)][1] - line[niu_clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (dn < 1e-6f) continue;
        dx /= dn;
        dy /= dn;
        if (line[i][1] + dx * dist > (float)(MT9V03X_H - 2)) continue;
        mid[mnum][0] = line[i][0] - dy * dist;
        mid[mnum][1] = line[i][1] + dx * dist;
        mnum++;
    }
    *num2 = mnum;
}

// ======================================================================
// 中线生成 — 右线推中线（对称）
// ======================================================================
void get_mid_R(float line[][2], int num, float mid[][2], int *num2, int approx_num)
{
    int mnum = 0;
    float dist = rood_wid / 2.0f;
    for (int i = 0; i < num; i++) {
        float dx = line[niu_clip(i + approx_num, 0, num - 1)][0] - line[niu_clip(i - approx_num, 0, num - 1)][0];
        float dy = line[niu_clip(i + approx_num, 0, num - 1)][1] - line[niu_clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (dn < 1e-6f) continue;
        dx /= dn;
        dy /= dn;
        if (line[i][1] - dx * dist > (float)(MT9V03X_H - 2)) continue;
        mid[mnum][0] = line[i][0] + dy * dist;
        mid[mnum][1] = line[i][1] - dx * dist;
        mnum++;
    }
    *num2 = mnum;
}

// ======================================================================
// 拐点检测 — 局部角度变化率
// ======================================================================
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0.0f;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[niu_clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[niu_clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[niu_clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[niu_clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);

        if (dn1 < 1e-6f || dn2 < 1e-6f) {
            angle_out[i] = 0.0f;
            continue;
        }
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// ======================================================================
// 拐点检测 — 窗口非极大抑制
// ======================================================================
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabsf(angle_in[niu_clip(i + j, 0, num - 1)]) > fabsf(angle_out[i])) {
                angle_out[i] = 0.0f;
                break;
            }
        }
    }
}

// ======================================================================
// 拐点 + 直道判定
// ======================================================================
void guai_mum(float Edge[][2], int num, float ang_out[], int *posi, int *if_zhi)
{
    conmax = 0.0f;
    *posi = 0;
    *if_zhi = 1;
    float ang_in[100];
    local_angle_points(Edge, num, ang_in, 5);
    nms_angle(ang_in, num, ang_out, 10);

    // --- 拐点检测（置信度滤波） ---
    for (int i = 0; i < num; i++) {
        if (ang_out[i] == 0.0f) continue;
        int im1 = niu_clip(i - 5, 0, num - 1);
        int ip1 = niu_clip(i + 5, 0, num - 1);
        float conf = fabsf(ang_out[i]) - (fabsf(ang_out[im1]) + fabsf(ang_out[ip1])) / 2.0f;
        if (conf > 1.05f && conf < 2.0f) {
            *posi = i;
        }
        if (*posi > 0) break;
    }

    // --- 最小二乘法判定直道 ---
    *if_zhi = 0;
    if (num >= 25) {
        float x1 = Edge[2][0];
        float y1 = Edge[2][1];
        float x2 = Edge[num - 3][0];
        float y2 = Edge[num - 3][1];

        float dx = x1 - x2;
        float dy = y2 - y1;
        if (fabsf(dx) < 1e-6f) { *if_zhi = 1; return; }

        float A = 1.0f / dx;
        float B = 1.0f / dy;
        float C = y2 / dy - x2 / dx;
        float D = 0.0f;

        for (int i = 3; i < num - 3; i++) {
            float x0 = Edge[i][0];
            float y0 = Edge[i][1];
            float numerator = fabsf(A * x0 + B * y0 + C);
            D += numerator * numerator / (A * A + B * B) / (float)(num - 6);
        }

        conmax = D;

        if (D <= 2.5f) {
            *if_zhi = 1;
        }
    }
}

// ======================================================================
// 逆透视 — 边线原地坐标变换
// ======================================================================
void touxian(float line[][2], int *num)
{
    for (int i = 0; i < *num; i++) {
        float x = Xrot_point(line[i][0], line[i][1]);
        float y = Yrot_point(line[i][0], line[i][1]);
        if (x >= 0.0f && x < (float)MT9V03X_W && y >= 0.0f && y < (float)MT9V03X_H) {
            line[i][0] = x;
            line[i][1] = y;
        } else {
            *num = i;
            break;
        }
    }
}

// ======================================================================
// 逆透视 — 整图映射（供显示）
// ======================================================================
void nitoushi(void)
{
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < MT9V03X_W; j++) {
            int x = (int)XShowRotimg((float)j, (float)i);
            int y = (int)YShowRotimg((float)j, (float)i);

            if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H) {
                if (image_use[y][x] > daj) {
                    show[i][j] = 255;
                } else {
                    show[i][j] = 0;
                }
            }
        }
    }
}

// ======================================================================
// 控制 — 纯跟踪转向
// ======================================================================
void get_error(void)
{
    int qianzhan = yumao;

    if (xunxian_dir == -1) {   // 巡左边线
        if (Lmidnum2 > qianzhan) {
            float dx = Lout_MID[qianzhan][0] - (float)(MT9V03X_W / 2 - 2);
            float dy = (float)(MT9V03X_H + 20 - 2) - Lout_MID[qianzhan][1];
            error = 1.0f * atan2f(dx, dy) * 180.0f / 3.14159265f;
        } else {
            float dx = Lout_MID[Lmidnum2 - 1][0] - (float)(MT9V03X_W / 2 - 2);
            float dy = (float)(MT9V03X_H + 20 - 2) - Lout_MID[Lmidnum2 - 1][1];
            error = 1.0f * atan2f(dx, dy) * 180.0f / 3.14159265f;
        }
    } else if (xunxian_dir == 1) {   // 巡右边线
        if (Rmidnum2 > qianzhan) {
            float dx = Rout_MID[qianzhan][0] - (float)(MT9V03X_W / 2 - 2);
            float dy = (float)(MT9V03X_H + 20 - 2) - Rout_MID[qianzhan][1];
            error = 1.0f * atan2f(dx, dy) * 180.0f / 3.14159265f;
        } else {
            float dx = Rout_MID[Rmidnum2 - 1][0] - (float)(MT9V03X_W / 2 - 2);
            float dy = (float)(MT9V03X_H + 20 - 2) - Rout_MID[Rmidnum2 - 1][1];
            error = 1.0f * atan2f(dx, dy) * 180.0f / 3.14159265f;
        }
    }

    // PD 舵机
    if ((xunxian_dir == -1 && Lout_count > 4) || (xunxian_dir == 1 && Rout_count > 4)) {
        duty = -P_fx * error - D_fx * (error - error_last);
    }
    error_last = error;

    // 限幅
    if (duty < -70.0f) duty = -70.0f;
    if (duty >  70.0f) duty =  70.0f;
}

// ======================================================================
// 控制 — 增量式 PID 速度环
// ======================================================================
void madasudu(void)
{
    L_err[2] = L_err[1];
    L_err[1] = L_err[0];
    L_err[0] = ((speedtobel - speedl) - dl);
    L_sum = speedtobel - speedl;
    dl = speedtobel - speedl;

    R_err[2] = R_err[1];
    R_err[1] = R_err[0];
    R_err[0] = ((speedtober - speedr) - dr);
    R_sum = speedtober - speedr;
    dr = speedtober - speedr;

    float PIDL = KP_L * L_err[0] + KI_L * L_sum + KD_L * (L_err[0] - L_err[1]);
    float PIDR = KP_R * R_err[0] + KI_R * R_sum + KD_R * (R_err[0] - R_err[1]);

    // 积分限幅
    if (PIDL > PID_I_LIMIT)  PIDL = (float)PID_I_LIMIT;
    if (PIDL < -PID_I_LIMIT) PIDL = -(float)PID_I_LIMIT;
    if (PIDR > PID_I_LIMIT)  PIDR = (float)PID_I_LIMIT;
    if (PIDR < -PID_I_LIMIT) PIDR = -(float)PID_I_LIMIT;

    pwml += PIDL;
    pwmr += PIDR;

    // 输出限幅
    if (pwml < -(float)PWM_LIMIT) pwml = -(float)PWM_LIMIT;
    if (pwml >  (float)PWM_LIMIT) pwml =  (float)PWM_LIMIT;
    if (pwmr < -(float)PWM_LIMIT) pwmr = -(float)PWM_LIMIT;
    if (pwmr >  (float)PWM_LIMIT) pwmr =  (float)PWM_LIMIT;
}

// ======================================================================
// 元素处理 — 坡道
// ======================================================================
void if_po(void)
{
    // if (dl1b_distance_mm <= 400 && lw <= 5 && podao == 0) {
    if (lw <= 5 && podao == 0) {   // TODO: 接入 TOF 距离传感器
        podao = 1;
        pochang = 0;
        poyanshi = 0;
    }
    if (podao == 1) {
        pochang += (int)(speedl + speedr);
        if (pochang >= 40000) {
            podao = 2;
        }
    }
    if (podao == 2) {
        poyanshi++;
        if (poyanshi >= 100) {
            podao = 0;
        }
    }
}

// ======================================================================
// 元素处理 — 减速
// ======================================================================
void jiansu(void)
{
    if (banmacishu >= 1 && jian == 0) {
        jianms++;
    }
    if (jianms >= 500) {
        jian = 1;
        jianms = 0;
    }
}

// ======================================================================
// 元素处理 — 斑马线/气泡检测
// ======================================================================
void if_banmaxian(void)
{
    int tiaobian = 0;
    for (int i = 5; i < 183; i++) {
        if (image_use[MT9V03X_H - 36][i] >= daj &&
            image_use[MT9V03X_H - 36][i + 1] < daj) {
            tiaobian++;
        }
    }
    if (tiaobian >= 7 && qipao == 0) {
        qipao = 1;
    }
    if (qipao == 1) {
        qipaojuli += (int)(speedl + speedr);
        if (qipaojuli >= 8000) {
            qipao = 0;
            qipaojuli = 0;
            banmacishu++;
        }
    }
}

// ======================================================================
// 元素处理 — 路障
// ======================================================================
void if_luzhang(void)
{
    if (banmacishu >= 1 && podao == 0 && qipao == 0 && luzhang == 0 &&
        yuanhuan == 0 && shizi == 0 && (ifzhi_L == 1 || ifzhi_R == 1)) {
        float Lx = 0.0f, Rx = 188.0f, lukuan = 0.0f;
        float h = (float)inv_rot_y(94.0f, 60.0f);
        h = 119.0f - h;
        for (int i = lwline; i > 0; i--) {
            if ((cbh(i, (int)((float)MT9V03X_H - h - 1), 6) == 1 &&
                 cbh(i + 1, (int)((float)MT9V03X_H - h - 1), 6) == 1 &&
                 cbh(i + 2, (int)((float)MT9V03X_H - h - 1), 6) == 1 &&
                 cbh(i + 3, (int)((float)MT9V03X_H - h - 1), 6) == 1 &&
                 cbh(i + 4, (int)((float)MT9V03X_H - h - 1), 6) == 0) ||
                i <= 8) {
                Lx = (float)(i + 4);
                break;
            }
        }
        for (int i = lwline; i < MT9V03X_W; i++) {
            if ((cbh(i, (int)((float)MT9V03X_H - h - 1), 2) == 1 &&
                 cbh(i - 1, (int)((float)MT9V03X_H - h - 1), 2) == 1 &&
                 cbh(i - 2, (int)((float)MT9V03X_H - h - 1), 2) == 1 &&
                 cbh(i - 3, (int)((float)MT9V03X_H - h - 1), 2) == 1 &&
                 cbh(i - 4, (int)((float)MT9V03X_H - h - 1), 2) == 0) ||
                i >= 180) {
                Rx = (float)(i - 4);
                break;
            }
        }
        lukuan = Xrot_point(Rx, (float)((float)MT9V03X_H - h - 1)) -
                 Xrot_point(Lx, (float)((float)MT9V03X_H - h - 1));
        if (lukuan <= 37.0f) {
            luzhang = 1;
            if (ifzhi_L == 1)
                dir_luzhang = 1;
            else if (ifzhi_R == 1)
                dir_luzhang = -1;
        }
    }

    if (luzhang == 1) {
        luzhangjuli += (int)(speedl + speedr);
        if (luzhangjuli >= 18000) {
            luzhang = 0;
            luzhangjuli = 0;
            dir_luzhang = 0;
        }
    }
}

// ======================================================================
// 元素处理 — 圆环
// ======================================================================
void if_yuanhuan(void)
{
    if (podao == 0 && luzhang == 0 && yuanhuan == 0 && shizi == 0) {
        if (ifzhi_L == 1 && ang_numR > 0 && ang_numR < 5) {
            yuanhuan = 1;
            dir_yuanhuan = 1;
            suo = 0;
        }

        if (ifzhi_R == 1 && ang_numL > 0 && ang_numL < 5) {
            yuanhuan = 1;
            dir_yuanhuan = -1;
            suo = 0;
        }
    }

    if (yuanhuan == 1) {
        juli += (int)(speedl + speedr);
        if (((dir_yuanhuan == 1 && Rout_count > 10 && ang_numR == 0) ||
             (dir_yuanhuan == -1 && Lout_count > 10 && ang_numL == 0)) &&
            juli > 14000) {
            yuanhuan = 2;
            juli = 0;
        }
    }

    if (yuanhuan == 2) {
        juli += (int)(speedl + speedr);
        if (juli > 20000) {
            if (ifzhi_L == 1 || ifzhi_R == 1) {
                yuanhuan = 3;
                juli = 0;
            }
        }
    }

    if (yuanhuan == 3) {
        juli += (int)(speedl + speedr);
        if (juli >= 20000) {
            yuanhuan = 0;
            yanshi = 0;
            juli = 0;
        }
    }
}

// ======================================================================
// 元素处理 — 十字
// ======================================================================
void if_shizi(void)
{
    if (podao == 0 && luzhang == 0 && yuanhuan == 0 && shizi == 0) {
        if ((ang_numR > 0 && ang_numR < 5 && Lout_count < 8) ||
            (ang_numL > 0 && ang_numL < 5 && Rout_count < 8) ||
            (ang_numR > 0 && ang_numR < 5 && ang_numL > 0 && ang_numL < 5)) {
            shizi = 1;
        }
    }

    if (shizi == 1) {
        len_shizi += (int)(speedl + speedr);
    }

    if (len_shizi > 14000) {
        len_shizi = 0;
        shizi = 0;
    }

    if (shizi == 1) {
        int h = 0;
        if (ang_numR > 0) {
            h = inv_rot_y((int)Ruse_edge[ang_numR][0],
                          (int)(Ruse_edge[ang_numR][1] - 55.0f));
            h = 119 - h;
        } else if (ang_numL > 0) {
            h = inv_rot_y((int)Luse_edge[ang_numL][0],
                          (int)(Luse_edge[ang_numL][1] - 55.0f));
            h = 119 - h;
        } else {
            h = inv_rot_y(94, 119 - 50);
            h = 119 - h;
        }

        get_BLY(L_edge, &L_count, R_edge, &R_count, h);

        // 逆透视
        touxian(L_edge, &L_count);
        touxian(R_edge, &R_count);

        // 滤波
        blur_points(L_edge, Lout_edge, L_count, 3);
        blur_points(R_edge, Rout_edge, R_count, 3);

        // 重采样
        Lout_count = L_count * 2;
        Rout_count = R_count * 2;
        resample_points2(L_edge, L_count, Luse_edge, &Lout_count, 3);
        resample_points2(R_edge, R_count, Ruse_edge, &Rout_count, 3);
    }
}

// ======================================================================
// 主搜索调度
// ======================================================================
void search_bianxian(void)
{
    if (qipao == 0) {
        get_BLY(L_edge, &L_count, R_edge, &R_count, 40);

        // 逆透视
        touxian(L_edge, &L_count);
        touxian(R_edge, &R_count);

        // 滤波
        blur_points(L_edge, Lout_edge, L_count, 3);
        blur_points(R_edge, Rout_edge, R_count, 3);

        // 重采样
        Lout_count = L_count * 2;
        Rout_count = R_count * 2;
        resample_points2(L_edge, L_count, Luse_edge, &Lout_count, 3);
        resample_points2(R_edge, R_count, Ruse_edge, &Rout_count, 3);

        // 找拐点
        guai_mum(Luse_edge, Lout_count, ang_L, &ang_numL, &ifzhi_L);
        guai_mum(Ruse_edge, Rout_count, ang_R, &ang_numR, &ifzhi_R);
        if (ang_numL > 0) { Lout_count = ang_numL; }
        if (ang_numR > 0) { Rout_count = ang_numR; }

    } else if (qipao == 1) {
        int h = inv_rot_y(94, 100);
        h = 119 - h;
        get_BLY(L_edge, &L_count, R_edge, &R_count, h);

        // 逆透视
        touxian(L_edge, &L_count);
        touxian(R_edge, &R_count);

        // 滤波
        blur_points(L_edge, Lout_edge, L_count, 3);
        blur_points(R_edge, Rout_edge, R_count, 3);

        // 重采样
        Lout_count = L_count * 2;
        Rout_count = R_count * 2;
        resample_points2(L_edge, L_count, Luse_edge, &Lout_count, 3);
        resample_points2(R_edge, R_count, Ruse_edge, &Rout_count, 3);

        // 找拐点
        guai_mum(Luse_edge, Lout_count, ang_L, &ang_numL, &ifzhi_L);
        guai_mum(Ruse_edge, Rout_count, ang_R, &ang_numR, &ifzhi_R);
        if (ang_numL > 0) { Lout_count = ang_numL; }
        if (ang_numR > 0) { Rout_count = ang_numR; }
    }
}

// ======================================================================
// 桥接函数：摄像头灰度图 → 牛爷爷全管线（视觉 + 控制）
// 供 image_proc_handler() 每帧调用
// ======================================================================
void niu_vision_pipeline(uint8 *gray_ptr, int img_w, int img_h)
{
    // 1. 灰度图缩放/裁剪到 MT9V03X 尺寸 → 一维数组供 OTSU
    static uint8 gray_buf[MT9V03X_H * MT9V03X_W];

    // 简单居中裁剪（假设输入尺寸 ≥ 目标尺寸）
    int off_x = (img_w > MT9V03X_W) ? (img_w - MT9V03X_W) / 2 : 0;
    int off_y = (img_h > MT9V03X_H) ? (img_h - MT9V03X_H) / 2 : 0;
    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            int sx = off_x + x;
            int sy = off_y + y;
            if (sx < img_w && sy < img_h)
                gray_buf[y * MT9V03X_W + x] = gray_ptr[sy * img_w + sx];
            else
                gray_buf[y * MT9V03X_W + x] = 0;
        }
    }

    // 2. OTSU 阈值
    daj = otsuThreshold(gray_buf);

    // 3. 二值化填 image_use[][]
    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            image_use[y][x] = (gray_buf[y * MT9V03X_W + x] > daj) ? 255 : 0;
        }
    }

    // 4. 最长白列 → 八邻域爬线
    get_highest();

    // 5. 爬线 + 逆透视 + 滤波 + 重采样 + 拐点
    search_bianxian();

    // 6. 元素检测
    if_banmaxian();
    if_po();
    jiansu();
    if_yuanhuan();
    if_shizi();
    if_luzhang();

    // 7. 中线生成
    get_mid_L(Luse_edge, Lout_count, Lin_MID,  &Lmidnum,  15);
    get_mid_R(Ruse_edge, Rout_count, Rin_MID,  &Rmidnum,  15);
    blur_points(Lin_MID,  Lout_MID,  Lmidnum,  3);
    blur_points(Rin_MID,  Rout_MID,  Rmidnum,  3);
    Lmidnum2 = Lmidnum;
    Rmidnum2 = Rmidnum;

    // 8. 巡线策略
    xunxain_celue();

    // 9. 逆透视显示（鸟瞰图 → show[][]，供 IPS200 显示）
    nitoushi();

    // 10. 纯跟踪 → error
    get_error();
}

// ======================================================================
// 巡线策略
// ======================================================================
void xunxain_celue(void)
{
    // 最长中线优先
    if (Lmidnum2 > Rmidnum2) {
        xunxian_dir = -1;
    } else {
        xunxian_dir = 1;
    }

    // 圆环策略
    if (yuanhuan == 1) {
        xunxian_dir = -dir_yuanhuan;
    }

    if (yuanhuan == 2) {
        if (dir_yuanhuan == 1) {
            if (Rout_count > 3) {
                xunxian_dir = dir_yuanhuan;
            } else if (ang_numL == 0 && suo == 0) {
                xunxian_dir = -dir_yuanhuan;
            } else {
                suo = 1;
                xunxian_dir = dir_yuanhuan;
            }
        }

        if (dir_yuanhuan == -1) {
            if (Lout_count > 5) {
                xunxian_dir = dir_yuanhuan;
            } else if (ang_numR == 0 && suo == 0) {
                xunxian_dir = -dir_yuanhuan;
            } else {
                suo = 1;
                xunxian_dir = dir_yuanhuan;
            }
        }
    }

    if (yuanhuan == 3) {
        xunxian_dir = -dir_yuanhuan;
    }

    // 路障策略
    if (luzhang == 1) {
        xunxian_dir = -dir_luzhang;
        if (xunxian_dir == 1) {
            Rout_MID[yumao][0] += 20.0f;
        } else if (xunxian_dir == -1) {
            Lout_MID[yumao][0] -= 20.0f;
        }
    }
}