#include "imgproc.hpp"
#include <cstdint>
#include <vector>
#include <unistd.h>

using namespace std;
using namespace cv;

// ========================================== 全局图像数组 (160x120) ==========================================
uint8_t Cramp_image[IPSH][IPSW];       // 压缩后的灰度图像
uint8_t heheImage[IPSH][IPSW];         // 二值化图像 (0=黑/赛道, 1=白/赛道边界外)
uint8_t color_image[IPSH][IPSW][3];    // 彩色图像 BGR

// ========================================== 红色物块检测变量 ==========================================
int red_block_detected = 0;
int red_block_center_x = -1;
int red_block_center_y = -1;
int extract_region_x = -1;
int extract_region_y = -1;
int extract_region_size = 64;

// ========================================== NCNN 模型 (已禁用) ==========================================
static int ncnn_initialized = 0;
static std::string last_classification_result = "";
static int consecutive_detect_count = 0;
static std::string confirmed_classification = "";

// ========================================== 扫线全局变量 ==========================================
static uint8_t* StoreSingleLine;
static int Ysite = 0, Xsite = 0;
static int BottomBorderRight = 158;       // 第119行右边界 (原 79 → 158)
static int BottomBorderLeft = 0;          // 第119行左边界
static int BottomCenter = 0;              // 第119行中点
RowAttributetypedef RowAttribute[IPSH];   // 每行边界信息 (120行)
static uint8_t ExtenLFlag = 0;
static uint8_t ExtenRFlag = 0;
static int IntervalLow = 0, IntervalHigh = 0;
int ImageScanInterval;                    // 扫边范围
int ImageScanInterval_Cross;              // 十字扫线范围
static int ytemp = 0;
static int TFSite = 0, left_FTSite = 0, right_FTSite = 0;
static float DetR = 0, DetL = 0;
ImageParametertypedef ImageParameter;
SystemDatatypdef SystemData;

// ========================================== 前瞻权重 (不变, 分辨率无关) ==========================================
float Weighting[10] = { 0.96, 0.92, 0.88, 0.83, 0.77, 0.59, 0.65, 0.59, 0.53, 0.59 };

// ========================================== 外部/全局变量 ==========================================
uint32_t encoder_distance = 0, ramp_distance = 0, stop_distance = 0;
uint32_t Cross_TPoint = 44;              // 原 22 → 44
uint32_t Rings_TPoint = 0;
uint32_t Straight_TPoint = 44;           // 原 22 → 44
uint8_t Ring_Help_Flag = 0;
int Repair_Point_Xsite, Repair_Point_Ysite;
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite;
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;
int Point_Xsite, Point_Ysite;

// ========================================== 赛道半宽数组 (120行, 60行→120行插值) ==========================================
// 原 60 行: {3,3,3,3,3,3,5,5,6,6,6,6,6,6,7,7,7,7,8,8,8,8,9,9,10,10,11,11,11,11,
//            12,12,13,13,14,14,15,15,15,15,16,16,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,24,24}
// 扩展到 120 行: 每个原值复制为相邻两行
uint8_t Half_Road_Wide[IPSH] = {
    3,3,  3,3,  3,3,  3,3,  3,3,  3,3,   // 0-11   (原 0-5 ×2)
    5,5,  5,5,                               // 12-15  (原 6-7 ×2)
    6,6,  6,6,  6,6,  6,6,  6,6,  6,6,     // 16-27  (原 8-13 ×2)
    7,7,  7,7,  7,7,  7,7,                  // 28-35  (原 14-17 ×2)
    8,8,  8,8,  8,8,  8,8,                  // 36-43  (原 18-21 ×2)
    9,9,  9,9,                              // 44-47  (原 22-23 ×2)
    10,10, 10,10,                            // 48-51  (原 24-25 ×2)
    11,11, 11,11, 11,11, 11,11,             // 52-59  (原 26-29 ×2)
    12,12, 12,12,                            // 60-63  (原 30-31 ×2)
    13,13, 13,13,                            // 64-67  (原 32-33 ×2)
    14,14, 14,14,                            // 68-71  (原 34-35 ×2)
    15,15, 15,15, 15,15, 15,15,             // 72-79  (原 36-39 ×2)
    16,16, 16,16, 16,16, 16,16,             // 80-87  (原 40-43 ×2)
    17,17, 17,17,                            // 88-91  (原 44-45 ×2)
    18,18, 18,18,                            // 92-95  (原 46-47 ×2)
    19,19, 19,19,                            // 96-99  (原 48-49 ×2)
    20,20, 20,20,                            // 100-103(原 50-51 ×2)
    21,21, 21,21,                            // 104-107(原 52-53 ×2)
    22,22, 22,22,                            // 108-111(原 54-55 ×2)
    23,23, 23,23,                            // 112-115(原 56-57 ×2)
    24,24, 24,24                             // 116-119(原 58-59 ×2)
};

// ========================================== onto 输出接口 ==========================================
float onto = 0.0f;
float angle_compensation = 0.0f;

// ========================================== zouma 兼容图像变量 ==========================================
cv::Mat frame_color;
cv::Mat frame_gray;
cv::Mat frame_bin;
uint8_t* img_gray = nullptr;
uint8_t bin_img_data[IPSW * IPSH];

// ========================================== 调试变量 ==========================================
float variance, variance_acc;
float avg_angle;
uint8_t all_block_size = 7;
uint8_t start_thre = 130;

// ========================================== OTSU 大津法阈值计算 ==========================================
uint8_t Threshold_deal(uint8_t* image,
    uint16_t col,
    uint16_t row,
    uint32_t pixel_threshold) {
#define GrayScale 256
    uint16_t width = col;
    uint16_t height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8_t threshold = 0;
    uint8_t* data = image;
    for (i = 0; i < GrayScale; i++) {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32_t gray_sum = 0;
    for (i = 0; i < height; i += 1) {
        for (j = 0; j < width; j += 1) {
            pixelCount[(int)data[i * width + j]]++;
            gray_sum += (int)data[i * width + j];
        }
    }

    for (i = 0; i < GrayScale; i++) {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_threshold; j++) {
        w0 += pixelPro[j];
        u0tmp += j * pixelPro[j];
        w1 = 1 - w0;
        // OTSU 除法保护: w0, w1 可能为 0
        if (w0 < 0.0001f || w1 < 0.0001f) continue;
        u1tmp = gray_sum / pixelSum - u0tmp;
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax) {
            deltaMax = deltaTmp;
            threshold = (uint8_t)j;
        }
        if (deltaTmp < deltaMax) {
            break;
        }
    }
    return threshold;
}

// ========================================== 阈值分离与二值化 ==========================================
void Thershold_separation_Otsu(void) {
    ImageParameter.Threshold =
        (uint8_t)Threshold_deal(Cramp_image[0], IPSW, IPSH, (uint32_t)ImageParameter.Threshold_detach);
    if (ImageParameter.Threshold < ImageParameter.Threshold_static)
        ImageParameter.Threshold = (uint8_t)ImageParameter.Threshold_static;
    uint8_t m, n;
    uint8_t ther;
    for (m = 0; m < IPSH; m++) {
        for (n = 0; n < IPSW; n++) {
            // 边缘区域补偿防止图像不均匀 (原 n≤15 → n≤30, n≥65→n≥130)
            if (n <= 30)
                ther = ImageParameter.Threshold - 10;
            else if ((n > 140 && n <= 150))
                ther = ImageParameter.Threshold - 10;
            else if (n >= 130)
                ther = ImageParameter.Threshold - 10;
            else
                ther = ImageParameter.Threshold;
            /*二值化*/
            if (Cramp_image[m][n] > (ther))
                heheImage[m][n] = 1;  // 白
            else
                heheImage[m][n] = 0;  // 黑
        }
    }
}

// ========================================== 跳变点检测 ==========================================
void JumpPointAndType(uint8_t* p, uint8_t type, int L, int H, JumpPointtypedef* Q) {
    int i = 0;
    if (type == 'L') {
        for (i = H; i >= L; i--) {
            if (*(p + i) == 1 && *(p + i - 1) != 1) {
                Q->point = i;
                Q->type = 'T';
                break;
            }
            else if (i == (L + 1)) {
                if (*(p + (L + H) / 2) != 0) {
                    Q->point = (L + H) / 2;
                    Q->type = 'W';
                    break;
                }
                else {
                    Q->point = H;
                    Q->type = 'H';
                    break;
                }
            }
        }
    }
    else if (type == 'R') {
        for (i = L; i <= H; i++) {
            if (*(p + i) == 1 && *(p + i + 1) != 1) {
                Q->point = i;
                Q->type = 'T';
                break;
            }
            else if (i == (H - 1)) {
                if (*(p + (L + H) / 2) != 0) {
                    Q->point = (L + H) / 2;
                    Q->type = 'W';
                    break;
                }
                else {
                    Q->point = L;
                    Q->type = 'H';
                    break;
                }
            }
        }
    }
}

// ========================================== 底部基线扫描 (第 119 行 + 118→110) ==========================================
static uint8_t DrawLinesBasic(void) {
    /* 单独扫第 119 行 (原 59 → 119) */
    StoreSingleLine = heheImage[119];
    if (*(StoreSingleLine + PictureCentring) == 1 || *(StoreSingleLine + PictureCentring) == 0) {
        // 搜索右边线
        BottomBorderRight = 78;  // 默认值 (原 39 → 78)
        for (Xsite = 159; Xsite > PictureCentring; Xsite--) {
            if (*(StoreSingleLine + Xsite) == 0 &&
                *(StoreSingleLine + Xsite - 1) == 1) {
                BottomBorderRight = Xsite;
                break;
            }
        }
        if (BottomBorderRight == 78) {
            for (Xsite = 159; Xsite > PictureCentring; Xsite--) {
                if (*(StoreSingleLine + Xsite) == 1 &&
                    *(StoreSingleLine + Xsite - 1) == 1) {
                    BottomBorderRight = Xsite;
                    break;
                }
            }
        }
        // 搜索左边线
        BottomBorderLeft = 78;
        for (Xsite = 0; Xsite < PictureCentring; Xsite++) {
            if (*(StoreSingleLine + Xsite) == 0 &&
                *(StoreSingleLine + Xsite + 1) == 1) {
                BottomBorderLeft = Xsite;
                break;
            }
        }
        if (BottomBorderLeft == 78) {
            for (Xsite = 0; Xsite < PictureCentring; Xsite++) {
                if (*(StoreSingleLine + Xsite) == 1 &&
                    *(StoreSingleLine + Xsite + 1) == 1) {
                    BottomBorderLeft = Xsite;
                    break;
                }
            }
        }
    }
    BottomCenter = (BottomBorderLeft + BottomBorderRight) / 2;
    RowAttribute[119].LeftBorder = BottomBorderLeft;
    RowAttribute[119].RightBorder = BottomBorderRight;
    RowAttribute[119].Center = BottomCenter;
    RowAttribute[119].Wide = BottomBorderRight - BottomBorderLeft;
    RowAttribute[119].IsLeftFind = 'T';
    RowAttribute[119].IsRightFind = 'T';

    /* 第 118→110 行扫描 (原 58→54, 共 9 行 → 118→110, 共 9 行) */
    for (Ysite = 118; Ysite > 110; Ysite--) {
        StoreSingleLine = heheImage[Ysite];
        // 扫右边线
        for (Xsite = 159; Xsite > RowAttribute[Ysite + 1].Center; Xsite--) {
            if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite - 1) == 1) {
                RowAttribute[Ysite].RightBorder = Xsite;
                break;
            }
        }
        if (Xsite <= RowAttribute[Ysite + 1].Center) {
            for (Xsite = 159; Xsite > RowAttribute[Ysite + 1].Center; Xsite--) {
                if (*(StoreSingleLine + Xsite) == 1 && *(StoreSingleLine + Xsite - 1) == 1) {
                    RowAttribute[Ysite].RightBorder = Xsite;
                    break;
                }
            }
        }
        // 扫左边线
        for (Xsite = 0; Xsite < RowAttribute[Ysite + 1].Center; Xsite++) {
            if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite + 1) == 1) {
                RowAttribute[Ysite].LeftBorder = Xsite;
                break;
            }
        }
        if (Xsite >= RowAttribute[Ysite + 1].Center) {
            for (Xsite = 0; Xsite < RowAttribute[Ysite + 1].Center; Xsite++) {
                if (*(StoreSingleLine + Xsite) == 1 && *(StoreSingleLine + Xsite + 1) == 1) {
                    RowAttribute[Ysite].LeftBorder = Xsite;
                    break;
                }
            }
        }
        RowAttribute[Ysite].IsLeftFind = 'T';
        RowAttribute[Ysite].IsRightFind = 'T';
        RowAttribute[Ysite].Center =
            (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
        RowAttribute[Ysite].Wide =
            RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
    }
    return 'T';
}

// ========================================== 边线追逐 (从第 110 行往上前进) ==========================================
static void DrawLinesProcess(void) {
    uint8_t L_Found_T = 'F';
    uint8_t Get_L_line = 'F';
    uint8_t R_Found_T = 'F';
    uint8_t Get_R_line = 'F';
    float D_L = 0;
    float D_R = 0;
    int ytemp_W_L;
    int ytemp_W_R;
    ExtenRFlag = 0;
    ExtenLFlag = 0;

    for (Ysite = 110; Ysite > ImageParameter.OFFLine; Ysite--) {
        ImageScanInterval_Cross = (110 - Ysite) / 2 + 5;
        StoreSingleLine = heheImage[Ysite];
        JumpPointtypedef JumpPoint[2];

        /****************************** 扫描右边线 ******************************/
        if (ImageParameter.WhiteLine < 5) {
            IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval;
            IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval;
        }
        else {
            IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval_Cross;
            IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval_Cross;
        }
        LimitL(IntervalLow);
        LimitH(IntervalHigh);
        JumpPointAndType(StoreSingleLine, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

        /****************************** 扫描左边线 ******************************/
        if (ImageParameter.WhiteLine < 5) {
            IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval;
            IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval;
        }
        else {
            IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval_Cross;
            IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval_Cross;
        }
        LimitL(IntervalLow);
        LimitH(IntervalHigh);
        JumpPointAndType(StoreSingleLine, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

        if (JumpPoint[0].type == 'W') {
            RowAttribute[Ysite].LeftBorder = RowAttribute[Ysite + 1].LeftBorder;
        }
        else {
            RowAttribute[Ysite].LeftBorder = JumpPoint[0].point;
        }

        if (JumpPoint[1].type == 'W') {
            RowAttribute[Ysite].RightBorder = RowAttribute[Ysite + 1].RightBorder;
        }
        else {
            RowAttribute[Ysite].RightBorder = JumpPoint[1].point;
        }

        RowAttribute[Ysite].IsLeftFind = JumpPoint[0].type;
        RowAttribute[Ysite].IsRightFind = JumpPoint[1].type;

        /************************************ 处理大跳变 (H类) *************************************/
        if ((RowAttribute[Ysite].IsLeftFind == 'H' || RowAttribute[Ysite].IsRightFind == 'H')) {
            if (RowAttribute[Ysite].IsLeftFind == 'H') {
                for (Xsite = (RowAttribute[Ysite].LeftBorder + 1);
                    Xsite <= (RowAttribute[Ysite].RightBorder - 1); Xsite++) {
                    if ((*(StoreSingleLine + Xsite) == 0) && (*(StoreSingleLine + Xsite + 1) != 0)) {
                        RowAttribute[Ysite].LeftBorder = Xsite;
                        RowAttribute[Ysite].IsLeftFind = 'T';
                        break;
                    }
                    else if (*(StoreSingleLine + Xsite) != 0)
                        break;
                    else if (Xsite == (RowAttribute[Ysite].RightBorder - 1)) {
                        RowAttribute[Ysite].LeftBorder = Xsite;
                        RowAttribute[Ysite].IsLeftFind = 'T';
                        break;
                    }
                }
            }
            if (RowAttribute[Ysite].IsRightFind == 'H') {
                for (Xsite = (RowAttribute[Ysite].RightBorder - 1);
                    Xsite >= (RowAttribute[Ysite].LeftBorder + 1); Xsite--) {
                    if ((*(StoreSingleLine + Xsite) == 0) && (*(StoreSingleLine + Xsite - 1) != 0)) {
                        RowAttribute[Ysite].RightBorder = Xsite;
                        RowAttribute[Ysite].IsRightFind = 'T';
                        break;
                    }
                    else if (*(StoreSingleLine + Xsite) != 0)
                        break;
                    else if (Xsite == (RowAttribute[Ysite].LeftBorder + 1)) {
                        RowAttribute[Ysite].RightBorder = Xsite;
                        RowAttribute[Ysite].IsRightFind = 'T';
                        break;
                    }
                }
            }
        }

        /************************************ 无边行处理 ************************************/
        int ysite = 0;
        uint8_t L_found_point = 0;
        uint8_t R_found_point = 0;

        // 处理右边线无边
        if (RowAttribute[Ysite].IsRightFind == 'W' && Ysite > 20 && Ysite < 100) {
            if (Get_R_line == 'F') {
                Get_R_line = 'T';
                ytemp_W_R = Ysite + 2;
                for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
                    if (RowAttribute[ysite].IsRightFind == 'T') {
                        R_found_point++;
                    }
                }
                if (R_found_point > 8) {
                    D_R = ((float)(RowAttribute[Ysite + R_found_point].RightBorder -
                        RowAttribute[Ysite + 3].RightBorder)) / ((float)(R_found_point - 3));
                    if (D_R > 0) {
                        R_Found_T = 'T';
                    }
                    else {
                        R_Found_T = 'F';
                        if (D_R < 0) {
                            ExtenRFlag = 'F';
                        }
                    }
                }
            }
            if (R_Found_T == 'T') {
                RowAttribute[Ysite].RightBorder =
                    RowAttribute[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
            }
            LimitL(RowAttribute[Ysite].RightBorder);
            LimitH(RowAttribute[Ysite].RightBorder);
        }

        // 处理左边线无边
        if (RowAttribute[Ysite].IsLeftFind == 'W' && Ysite > 20 && Ysite < 100) {
            if (Get_L_line == 'F') {
                Get_L_line = 'T';
                ytemp_W_L = Ysite + 2;
                for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
                    if (RowAttribute[ysite].IsLeftFind == 'T') {
                        L_found_point++;
                    }
                }
                if (L_found_point > 8) {
                    D_L = ((float)(RowAttribute[Ysite + 3].LeftBorder -
                        RowAttribute[Ysite + L_found_point].LeftBorder)) /
                        ((float)(L_found_point - 3));
                    if (D_L > 0) {
                        L_Found_T = 'T';
                    }
                    else {
                        L_Found_T = 'F';
                        if (D_L < 0) {
                            ExtenLFlag = 'F';
                        }
                    }
                }
            }
            if (L_Found_T == 'T') {
                RowAttribute[Ysite].LeftBorder =
                    RowAttribute[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
            }
            LimitL(RowAttribute[Ysite].LeftBorder);
            LimitH(RowAttribute[Ysite].LeftBorder);
        }

        /************************************ 数据整定 ************************************/
        if (RowAttribute[Ysite].IsLeftFind == 'W' && RowAttribute[Ysite].IsRightFind == 'W') {
            ImageParameter.WhiteLine++;
        }
        if (RowAttribute[Ysite].IsLeftFind == 'W' && Ysite < 110) {
            ImageParameter.Miss_Left_lines++;
        }
        if (RowAttribute[Ysite].IsRightFind == 'W' && Ysite < 110) {
            ImageParameter.Miss_Right_lines++;
        }

        LimitL(RowAttribute[Ysite].LeftBorder);
        LimitH(RowAttribute[Ysite].LeftBorder);
        LimitL(RowAttribute[Ysite].RightBorder);
        LimitH(RowAttribute[Ysite].RightBorder);

        RowAttribute[Ysite].Wide =
            RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
        RowAttribute[Ysite].Center =
            (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;

        if (RowAttribute[Ysite].Wide <= 12) {  // 原 6 → 12
            ImageParameter.OFFLine = Ysite + 1;
            break;
        }
        else if (RowAttribute[Ysite].RightBorder <= 20 || RowAttribute[Ysite].LeftBorder >= 140) {
            // 原 10 → 20, 70 → 140
            ImageParameter.OFFLine = Ysite + 1;
            break;
        }
    }
    return;
}

// ========================================== 八邻域扫线 (上交大左右手法则) ==========================================
void Search_Bottom_Line_OTSU(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline) {
    for (int Xsite = Col / 2 - 2; Xsite > 1; Xsite--) {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0) {
            RowAttribute[Bottonline].LeftBoundary = Xsite;
            break;
        }
    }
    for (int Xsite = Col / 2 + 2; Xsite < IPSW - 1; Xsite++) {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0) {
            RowAttribute[Bottonline].RightBoundary = Xsite;
            break;
        }
    }
}

void Search_Left_and_Right_Lines(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline) {
    int Left_Rule[2][8] = {
        {0,-1,1,0,0,1,-1,0},
        {-1,-1,1,-1,1,1,-1,1}
    };
    int Right_Rule[2][8] = {
        {0,-1,1,0,0,1,-1,0},
        {1,-1,1,1,-1,1,-1,-1}
    };
    int num = 0;
    uint8_t Left_Ysite = Bottonline;
    uint8_t Left_Xsite = (uint8_t)RowAttribute[Bottonline].LeftBoundary;
    uint8_t Left_Rirection = 0;
    uint8_t Pixel_Left_Ysite = Bottonline;
    uint8_t Pixel_Left_Xsite = 0;

    uint8_t Right_Ysite = Bottonline;
    uint8_t Right_Xsite = (uint8_t)RowAttribute[Bottonline].RightBoundary;
    uint8_t Right_Rirection = 0;
    uint8_t Pixel_Right_Ysite = Bottonline;
    uint8_t Pixel_Right_Xsite = 0;
    uint8_t Ysite = Bottonline;
    ImageParameter.OFFLineBoundary = 10;  // 原 5 → 10
    while (1) {
        num++;
        if (num > 400) {
            ImageParameter.OFFLineBoundary = Ysite;
            break;
        }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite) {
            if (Ysite < ImageParameter.OFFLineBoundary) {
                ImageParameter.OFFLineBoundary = Ysite;
                break;
            }
            else {
                Ysite--;
            }
        }
        /********* 左边巡线 *******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary) {
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) {
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else {
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) {
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0)
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else {
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0)
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }
            }
        }
        /********* 右边巡线 *******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary) {
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0) {
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else {
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0) {
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (RowAttribute[Right_Ysite].RightBoundary_First == 158)  // 原 79 → 158
                        RowAttribute[Right_Ysite].RightBoundary_First = Right_Xsite;
                    RowAttribute[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else {
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (RowAttribute[Right_Ysite].RightBoundary_First == 158)
                        RowAttribute[Right_Ysite].RightBoundary_First = Right_Xsite;
                    RowAttribute[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }
            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 6) {  // 原 3 → 6
            ImageParameter.OFFLineBoundary = Ysite;
            break;
        }
    }
}

void Search_Border_OTSU(uint8_t imageInput[IPSH][IPSW], uint8_t Row, uint8_t Col, uint8_t Bottonline) {
    ImageParameter.WhiteLine_L = 0;
    ImageParameter.WhiteLine_R = 0;
    for (int Xsite = 0; Xsite < IPSW; Xsite++) {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    for (int Ysite = 0; Ysite < IPSH; Ysite++) {
        RowAttribute[Ysite].LeftBoundary_First = 0;
        RowAttribute[Ysite].RightBoundary_First = 158;  // 原 79 → 158
        imageInput[Ysite][0] = 0;
        imageInput[Ysite][IPSW - 1] = 0;
    }
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);

    for (int Ysite = Bottonline; Ysite > ImageParameter.OFFLineBoundary + 1; Ysite--) {
        if (RowAttribute[Ysite].LeftBoundary < 6) {  // 原 3 → 6
            ImageParameter.WhiteLine_L++;
        }
        if (RowAttribute[Ysite].RightBoundary > IPSW - 6) {  // 原 3 → 6
            ImageParameter.WhiteLine_R++;
        }
    }
}

// ========================================== 寻找下拐点 ==========================================
int Find_Down_Point(int start, int end) {
    int i, t;
    int Right_Down_Find = 0;
    int Left_Down_Find = 0;
    int l_border[IPSH], r_border[IPSH];

    for (i = 0; i < IPSH; i++) {
        l_border[i] = RowAttribute[i].LeftBorder;
        r_border[i] = RowAttribute[i].RightBorder;
    }

    if (start < end) {
        t = start;
        start = end;
        end = t;
    }
    if (start >= IPSH - 1 - 10)  // 原 5 → 10
        start = IPSH - 1 - 10;
    if (end <= IPSH - 50)  // 原 25 → 50
        end = IPSH - 50;
    if (end <= 10)  // 原 5 → 10
        end = 10;

    for (i = start; i >= end; i--) {
        if (Left_Down_Find == 0 &&
            abs(l_border[i] - l_border[i + 1]) <= 10 &&  // 原 5 → 10
            abs(l_border[i + 1] - l_border[i + 2]) <= 10 &&
            abs(l_border[i + 2] - l_border[i + 3]) <= 10 &&
            (l_border[i] - l_border[i - 2]) >= 16 &&      // 原 8 → 16
            (l_border[i] - l_border[i - 3]) >= 30 &&      // 原 15 → 30
            (l_border[i] - l_border[i - 4]) >= 30) {
            Left_Down_Find = i;
        }
        if (Right_Down_Find == 0 &&
            abs(r_border[i] - r_border[i + 1]) <= 10 &&
            abs(r_border[i + 1] - r_border[i + 2]) <= 10 &&
            abs(r_border[i + 2] - r_border[i + 3]) <= 10 &&
            (r_border[i] - r_border[i - 2]) <= -16 &&
            (r_border[i] - r_border[i - 3]) <= -30 &&
            (r_border[i] - r_border[i - 4]) <= -30) {
            Right_Down_Find = i;
        }
        if (Left_Down_Find != 0 && Right_Down_Find != 0) {
            break;
        }
    }
    if (Left_Down_Find != 0 && Right_Down_Find != 0) {
        return 1;
    }
    else return 0;
}

// ========================================== 十字补线 ==========================================
void Get_ExtensionLine(void) {
    int Ysite = 110;  // 原 55 → 110
    float D_R = 0;
    float D_L = 0;
    int ytemp = 0;
    int WhiteLine = 0;
    int l_LostLine = 0, r_LostLine = 0;
    int OFFLine = ImageParameter.OFFLine;
    int MinLeftBd = 2;    // 原 1 → 2
    int MinRightBd = 157; // 原 78 → 157

    for (int y = 40; y < 110; y++) {  // 原 20→55 → 40→110
        if (RowAttribute[y].IsLeftFind == 'W') l_LostLine++;
        if (RowAttribute[y].IsRightFind == 'W') r_LostLine++;
    }

    if (l_LostLine < r_LostLine) WhiteLine = l_LostLine;
    else WhiteLine = r_LostLine;

    if (WhiteLine > 5) {
        /********************************* 左边补线 *********************************/
        left_FTSite = 0;
        TFSite = 110;
        for (Ysite = 100; Ysite >= (OFFLine + 4); Ysite--) {  // 原 50 → 100
            if (RowAttribute[Ysite].IsLeftFind == 'W') {
                while (Ysite >= (OFFLine + 4)) {
                    Ysite--;
                    if (RowAttribute[Ysite - 2].IsLeftFind == 'T' &&
                        RowAttribute[Ysite - 2].LeftBorder > 50 &&
                        RowAttribute[Ysite - 2].LeftBorder < 130) {
                        left_FTSite = Ysite - 2;
                        break;
                    }
                }
                D_L = (float)(RowAttribute[left_FTSite].LeftBorder - RowAttribute[TFSite].LeftBorder)
                    / ((float)(left_FTSite - TFSite));
                if (left_FTSite > OFFLine)
                    for (ytemp = TFSite; ytemp >= left_FTSite; ytemp--) {
                        RowAttribute[ytemp].LeftBorder = (int)(D_L * ((float)(ytemp - TFSite))) + RowAttribute[TFSite].LeftBorder;
                    }
            }
            else {
                TFSite = Ysite + 2;
            }
        }

        /********************************* 右边补线 *********************************/
        right_FTSite = 0;
        TFSite = 110;
        for (Ysite = 100; Ysite >= (OFFLine + 4); Ysite--) {
            if (RowAttribute[Ysite].IsRightFind == 'W') {
                while (Ysite >= (OFFLine + 4)) {
                    Ysite--;
                    if (RowAttribute[Ysite - 2].IsRightFind == 'T' &&
                        RowAttribute[Ysite - 2].RightBorder > 50 &&
                        RowAttribute[Ysite - 2].RightBorder < 150) {
                        right_FTSite = Ysite - 2;
                        break;
                    }
                }
                D_R = (float)(RowAttribute[right_FTSite].RightBorder - RowAttribute[TFSite].RightBorder)
                    / ((float)(right_FTSite - TFSite));
                if (right_FTSite > OFFLine)
                    for (ytemp = TFSite; ytemp >= right_FTSite; ytemp--) {
                        RowAttribute[ytemp].RightBorder = (int)(D_R * ((float)(ytemp - TFSite))) + RowAttribute[TFSite].RightBorder;
                    }
            }
            else
                TFSite = Ysite + 2;
        }

        // 重新计算中线和宽度
        for (int y = OFFLine; y <= 119; y++) {
            if (RowAttribute[y].LeftBorder >= 0 && RowAttribute[y].LeftBorder < IPSW &&
                RowAttribute[y].RightBorder >= 0 && RowAttribute[y].RightBorder < IPSW) {
                RowAttribute[y].Center = (RowAttribute[y].LeftBorder + RowAttribute[y].RightBorder) / 2;
                RowAttribute[y].Wide = RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
                if (RowAttribute[y].IsLeftFind == 'W' && RowAttribute[y].LeftBorder > MinLeftBd) {
                    RowAttribute[y].IsLeftFind = 'T';
                }
                if (RowAttribute[y].IsRightFind == 'W' && RowAttribute[y].RightBorder < MinRightBd) {
                    RowAttribute[y].IsRightFind = 'T';
                }
            }
        }
    }
}

// ========================================== 十字检测处理 ==========================================
void element_I() {
    int l_LostLine = 0, r_LostLine = 0;
    int zong_sign = 0;

    for (int y = 40; y < 110; y++) {  // 原 20→55 → 40→110
        if (RowAttribute[y].IsLeftFind == 'W') l_LostLine++;
        if (RowAttribute[y].IsRightFind == 'W') r_LostLine++;
    }

    if (l_LostLine > 4 && r_LostLine > 4 && zong_sign == 0) {
        Get_ExtensionLine();
    }
}

// ========================================== 中线平滑滤波 ==========================================
void RouteFilter(void) {
    for (Ysite = 116; Ysite >= (ImageParameter.OFFLine + 5); Ysite--) {  // 原 58 → 116
        if (RowAttribute[Ysite].IsLeftFind == 'W'
            && RowAttribute[Ysite].IsRightFind == 'W'
            && Ysite <= 90  // 原 45 → 90
            && RowAttribute[Ysite - 1].IsLeftFind == 'W'
            && RowAttribute[Ysite - 1].IsRightFind == 'W') {
            ytemp = Ysite;
            while (ytemp >= (ImageParameter.OFFLine + 5)) {
                ytemp--;
                if (RowAttribute[ytemp].IsLeftFind == 'T' && Xsite >= 40 && Xsite <= 80) {
                    if (RowAttribute[ytemp].IsRightFind == 'T' && Xsite >= 80 && Xsite <= 120) {
                        DetR = (float)(RowAttribute[ytemp - 1].Center - RowAttribute[Ysite + 2].Center)
                            / (float)(ytemp - 1 - Ysite - 2);
                        int CenterTemp = RowAttribute[Ysite + 2].Center;
                        int LineTemp = Ysite + 2;
                        while (Ysite >= ytemp) {
                            RowAttribute[Ysite].Center = (int)(CenterTemp + DetR * (float)(Ysite - LineTemp));
                            Ysite--;
                        }
                    }
                }
            }
        }
        RowAttribute[Ysite].Center =
            (RowAttribute[Ysite - 1].Center + 2 * RowAttribute[Ysite].Center) / 3;
    }
}

// ========================================== 直道加速检测 ==========================================
void Straightacc_Test(void) {
    int sum = 0;
    for (Ysite = 110; Ysite > ImageParameter.OFFLine + 1; Ysite--) {  // 原 55 → 110
        sum += (RowAttribute[Ysite].Center - PictureCentring) * (RowAttribute[Ysite].Center - PictureCentring);
    }
    variance_acc = (float)sum / (109 - ImageParameter.OFFLine);  // 原 54 → 109
    if (variance_acc < ImageParameter.variance_acc && ImageParameter.OFFLine <= 30) {  // 原 15 → 30
        ImageParameter.straight_acc = 1;
    }
    else
        ImageParameter.straight_acc = 0;
}

// ========================================== 动态前瞻 ==========================================
void Update_Dynamic_Lookahead(void) {
    ImageParameter.TowPoint = 20;  // 固定前瞻 (原值, 可调)
}

// ========================================== 直线判断 ==========================================
float Straight_Judge(uint8_t dir, uint8_t start, uint8_t end) {
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir) {
    case 1:
        k = (float)(RowAttribute[start].LeftBorder - RowAttribute[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++) {
            Err = (RowAttribute[start].LeftBorder + k * i - RowAttribute[i + start].LeftBorder) *
                (RowAttribute[start].LeftBorder + k * i - RowAttribute[i + start].LeftBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:
        k = (float)(RowAttribute[start].RightBorder - RowAttribute[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++) {
            Err = (RowAttribute[start].RightBorder + k * i - RowAttribute[i + start].RightBorder) *
                (RowAttribute[start].RightBorder + k * i - RowAttribute[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}

// ========================================== 二值图像滤波 ==========================================
void Bin_Image_Filter(void) {
    uint16_t nr;
    uint16_t nc;
    for (nr = 1; nr < IPSH - 1; nr++) {
        for (nc = 1; nc < IPSW - 1; nc = nc + 1) {
            if ((heheImage[nr][nc] == 0)
                && (heheImage[nr - 1][nc] + heheImage[nr + 1][nc]
                    + heheImage[nr][nc + 1] + heheImage[nr][nc - 1] > 2)) {
                heheImage[nr][nc] = 1;
            }
            else if ((heheImage[nr][nc] == 1)
                && (heheImage[nr - 1][nc] + heheImage[nr + 1][nc]
                    + heheImage[nr][nc + 1] + heheImage[nr][nc - 1] < 2)) {
                heheImage[nr][nc] = 0;
            }
        }
    }
}

// ========================================== 速度控制因子 ==========================================
void Speed_Control_Factor(void) {
    float SpeedGain = 0;
    SpeedGain = (SystemData.SpeedData.nowspeed - SystemData.SpeedData.MinSpeed) * 0.2 + 0.5;
    if (SpeedGain >= 3) SpeedGain = 3;
    else if (SpeedGain <= -1) SpeedGain = -1;
}

// ========================================== 左圆环检测 ==========================================
void Element_Judgment_Left_Rings() {
    if (ImageParameter.WhiteLine_R > 12     // 原 6 → 12
        || ImageParameter.WhiteLine_L < 4   // 原 2 → 4
        || ImageParameter.OFFLine > 40      // 原 20 → 40
        || Straight_Judge(1, 10, 110) > 2   // 原 5,55 → 10,110
        || ImageParameter.WhiteLine > 12    // 原 6 → 12
        || RowAttribute[108].IsLeftFind == 'W'     // 原 54 → 108
        || RowAttribute[110].IsLeftFind == 'W'     // 原 55 → 110
        || RowAttribute[112].IsLeftFind == 'W'     // 原 56 → 112
        || RowAttribute[114].IsLeftFind == 'W')    // 原 57 → 114
        return;

    int ring_ysite = 50;  // 原 25 → 50
    uint8_t Left_Less_Num = 0;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 116; Ysite > ring_ysite; Ysite--) {  // 原 58 → 116
        if (RowAttribute[Ysite].LeftBoundary_First - RowAttribute[Ysite - 1].LeftBoundary_First > 6) {  // 原 3 → 6
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 116; Ysite > ring_ysite; Ysite--) {
        if (RowAttribute[Ysite + 1].LeftBoundary - RowAttribute[Ysite].LeftBoundary > 6) {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > Left_RingsFlag_Point1_Ysite - 22; Ysite--) {  // 原 11 → 22
        if (Ysite <= 0) break;
        if (RowAttribute[Ysite].IsLeftFind == 'W')
            Left_Less_Num++;
    }
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageParameter.OFFLine; Ysite--) {
        if (RowAttribute[Ysite + 12].LeftBorder < RowAttribute[Ysite + 6].LeftBorder
            && RowAttribute[Ysite + 10].LeftBorder < RowAttribute[Ysite + 6].LeftBorder
            && RowAttribute[Ysite + 6].LeftBorder > RowAttribute[Ysite + 4].LeftBorder
            && RowAttribute[Ysite + 6].LeftBorder > RowAttribute[Ysite + 2].LeftBorder) {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2 && Ring_Help_Flag == 0) {  // 原 +1 → +2
        if (ImageParameter.Miss_Left_lines > 20)  // 原 10 → 20
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2) {
        ImageParameter.image_element_rings = 1;
        ImageParameter.image_element_rings_flag = 1;
        ImageParameter.ring_big_small = 1;
        cout << "left ring detected" << endl;
    }
    Ring_Help_Flag = 0;
}

// ========================================== 右圆环检测 ==========================================
void Element_Judgment_Right_Rings() {
    if (ImageParameter.WhiteLine_L > 6       // 原 3 → 6
        || ImageParameter.WhiteLine_R < 10   // 原 5 → 10
        || ImageParameter.OFFLine > 60       // 原 30 → 60
        || Straight_Judge(1, 30, 100) > 1    // 原 15,50 → 30,100
        || RowAttribute[108].IsRightFind == 'W'    // 原 54 → 108
        || RowAttribute[110].IsRightFind == 'W'    // 原 55 → 110
        || RowAttribute[112].IsRightFind == 'W'    // 原 56 → 112
        || RowAttribute[114].IsRightFind == 'W')   // 原 57 → 114
        return;

    int ring_ysite = 50;  // 原 25 → 50
    uint8_t Right_Less_Num = 0;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;

    for (int Ysite = 116; Ysite > ring_ysite; Ysite--) {
        if (RowAttribute[Ysite - 1].RightBoundary_First - RowAttribute[Ysite].RightBoundary_First > 6) {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 116; Ysite > ring_ysite; Ysite--) {
        if (RowAttribute[Ysite].RightBoundary - RowAttribute[Ysite + 1].RightBoundary > 6) {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > Right_RingsFlag_Point1_Ysite - 22; Ysite--) {
        if (Ysite <= 0) break;
        if (RowAttribute[Ysite].IsRightFind == 'W')
            Right_Less_Num++;
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageParameter.OFFLine; Ysite--) {
        if (RowAttribute[Ysite + 12].RightBorder > RowAttribute[Ysite + 6].RightBorder
            && RowAttribute[Ysite + 10].RightBorder > RowAttribute[Ysite + 6].RightBorder
            && RowAttribute[Ysite + 6].RightBorder < RowAttribute[Ysite + 4].RightBorder
            && RowAttribute[Ysite + 6].RightBorder < RowAttribute[Ysite + 2].RightBorder) {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 2 && Ring_Help_Flag == 0) {
        if (ImageParameter.Miss_Right_lines > 20)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 2) {
        ImageParameter.image_element_rings = 2;
        ImageParameter.image_element_rings_flag = 1;
        ImageParameter.ring_big_small = 1;
        cout << "right ring detected" << endl;
    }
    Ring_Help_Flag = 0;
}

// ========================================== 左圆环处理 ==========================================
void Element_Handle_Left_Rings() {
    int num = 0;
    for (int Ysite = 110; Ysite > 48; Ysite--) {  // 原 55→24 → 110→48
        if (RowAttribute[Ysite].IsLeftFind == 'W') num++;
        if (RowAttribute[Ysite + 6].IsLeftFind == 'W' && RowAttribute[Ysite + 4].IsLeftFind == 'W'
            && RowAttribute[Ysite + 2].IsLeftFind == 'W' && RowAttribute[Ysite].IsLeftFind == 'T')
            break;
    }
    // flag1→2: 准备进环
    if (ImageParameter.image_element_rings_flag == 1 && num > 40) {  // 原 20 → 40
        ImageParameter.image_element_rings_flag = 2;
    }
    // flag2→5: 进入圆环
    if (ImageParameter.image_element_rings_flag == 2 && num < 20) {  // 原 10 → 20
        ImageParameter.image_element_rings_flag = 5;
    }
    // flag5→6: 进环中
    if (ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_R > 32) {  // 原 16 → 32
        ImageParameter.image_element_rings_flag = 6;
    }
    // flag6→7: 环内
    if (ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_R < 10) {  // 原 5 → 10
        ImageParameter.image_element_rings_flag = 7;
    }
    // flag7→8: 环内 → 出环 (检测右边界凸点)
    if (ImageParameter.image_element_rings_flag == 7) {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 100; Ysite > ImageParameter.OFFLine + 14; Ysite--) {  // 原 50→7 → 100→14
            if (RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 4].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 4].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 2].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 2].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 8].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 8].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 12].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 12].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 10].RightBorder
                && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 10].RightBorder) {
                Point_Xsite = RowAttribute[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 44) {  // 原 22 → 44
            ImageParameter.image_element_rings_flag = 8;
        }
    }
    // flag8→9: 出环后
    if (ImageParameter.image_element_rings_flag == 8) {
        if (Straight_Judge(2, ImageParameter.OFFLine + 36, 100) < 1  // 原 18,50 → 36,100
            && ImageParameter.WhiteLine_R < 28                       // 原 14 → 28
            && ImageParameter.OFFLine < 24) {                        // 原 12 → 24
            ImageParameter.image_element_rings_flag = 9;
        }
    }
    // flag9→0: 结束圆环
    if (ImageParameter.image_element_rings_flag == 9) {
        int num = 0;
        for (int Ysite = 110; Ysite > 20; Ysite--) {  // 原 55→10 → 110→20
            if (RowAttribute[Ysite].IsLeftFind == 'W') num++;
        }
        if (num < 16) {  // 原 8 → 16
            ImageParameter.image_element_rings_flag = 0;
            ImageParameter.image_element_rings = 0;
            ImageParameter.ring_big_small = 0;
        }
    }

    printf("L_flag:%d num:%d\n", ImageParameter.image_element_rings_flag, num);

    /*************************************** 左环岛处理 **************************************/
    // flag 1-4: 半宽处理
    if (ImageParameter.image_element_rings_flag >= 1
        && ImageParameter.image_element_rings_flag <= 4) {
        for (int Ysite = 119; Ysite > ImageParameter.OFFLine; Ysite--) {
            RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
    // flag 5-6: 进环补线
    if (ImageParameter.image_element_rings_flag == 5
        || ImageParameter.image_element_rings_flag == 6) {
        int flag_Xsite_1 = 0;
        int flag_Ysite_1 = 0;
        float Slope_Rings = 0;
        for (Ysite = 110; Ysite > ImageParameter.OFFLine; Ysite--) {  // 原 55 → 110
            for (Xsite = RowAttribute[Ysite].LeftBorder + 1; Xsite < RowAttribute[Ysite].RightBorder - 1; Xsite++) {
                if (heheImage[Ysite][Xsite] == 1 && heheImage[Ysite][Xsite + 1] == 0) {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = Xsite;
                    Slope_Rings = (float)(220 - flag_Xsite_1) / (float)(119 - flag_Ysite_1);  // 原 110→220, 59→119
                    break;
                }
            }
            if (flag_Ysite_1 != 0) break;
        }
        if (flag_Ysite_1 == 0) {
            for (Ysite = ImageParameter.OFFLine + 1; Ysite < 60; Ysite++) {  // 原 30 → 60
                if (RowAttribute[Ysite].IsLeftFind == 'T' && RowAttribute[Ysite + 1].IsLeftFind == 'T'
                    && RowAttribute[Ysite + 2].IsLeftFind == 'W'
                    && abs(RowAttribute[Ysite].LeftBorder - RowAttribute[Ysite + 2].LeftBorder) > 20) {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = RowAttribute[flag_Ysite_1].LeftBorder;
                    ImageParameter.OFFLine = (uint8_t)Ysite;
                    Slope_Rings = (float)(220 - flag_Xsite_1) / (float)(119 - flag_Ysite_1);
                    break;
                }
            }
        }
        if (flag_Ysite_1 != 0) {
            int bottom_x = 0;
            for (int x = 159; x > 0; x--) {
                if (heheImage[119][x] == 0 && heheImage[119][x - 1] == 1) {
                    bottom_x = x;
                    break;
                }
            }
            if (bottom_x > 0) {
                Slope_Rings = (float)(bottom_x - flag_Xsite_1) / (float)(119 - flag_Ysite_1);
            }
            else {
                Slope_Rings = (float)(220 - flag_Xsite_1) / (float)(119 - flag_Ysite_1);
            }
            for (Ysite = flag_Ysite_1; Ysite < IPSH; Ysite++) {
                RowAttribute[Ysite].RightBorder = flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
                RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
            }
            RowAttribute[flag_Ysite_1].RightBorder = flag_Xsite_1;
            for (Ysite = flag_Ysite_1 - 1; Ysite > 20; Ysite--) {  // 原 10 → 20
                for (Xsite = RowAttribute[Ysite + 1].RightBorder - 20; Xsite < RowAttribute[Ysite + 1].RightBorder + 4; Xsite++) {
                    if (heheImage[Ysite][Xsite] == 1 && heheImage[Ysite][Xsite + 1] == 0) {
                        RowAttribute[Ysite].RightBorder = Xsite;
                        RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
                        RowAttribute[Ysite].Wide = RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
                        break;
                    }
                }
                if (RowAttribute[Ysite].Wide > 16 && RowAttribute[Ysite].RightBorder < RowAttribute[Ysite + 4].RightBorder) {
                    continue;
                }
                else {
                    ImageParameter.OFFLine = Ysite + 2;
                    break;
                }
            }
        }
    }
    // flag 7: 环内不额外处理
    // flag 8: 大圆环出环补线
    if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1) {
        Repair_Point_Xsite = 40;  // 原 20 → 40
        Repair_Point_Ysite = 10;  // 原 5 → 10
        for (int Ysite = 100; Ysite > 10; Ysite--) {
            if (heheImage[Ysite][64] == 1 && heheImage[Ysite - 1][64] == 0) {  // 原 32 → 64
                Repair_Point_Xsite = 64;
                Repair_Point_Ysite = Ysite - 1;
                ImageParameter.OFFLine = Ysite + 1;
                break;
            }
        }
        for (int Ysite = 110; Ysite > Repair_Point_Ysite - 6; Ysite--) {
            RowAttribute[Ysite].RightBorder = (RowAttribute[116].RightBorder - Repair_Point_Xsite) * (Ysite - 116) / (116 - Repair_Point_Ysite) + RowAttribute[116].RightBorder;
            RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
        }
    }
    // flag 8: 小圆环出环补线
    if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2) {
        Repair_Point_Xsite = 0;
        Repair_Point_Ysite = 0;
        for (int Ysite = 110; Ysite > 10; Ysite--) {
            if (heheImage[Ysite][30] == 1 && heheImage[Ysite - 1][30] == 0) {  // 原 15 → 30
                Repair_Point_Xsite = 30;
                Repair_Point_Ysite = Ysite - 1;
                ImageParameter.OFFLine = Ysite + 1;
                break;
            }
        }
        for (int Ysite = 110; Ysite > Repair_Point_Ysite - 6; Ysite--) {
            RowAttribute[Ysite].RightBorder = (RowAttribute[116].RightBorder - Repair_Point_Xsite) * (Ysite - 116) / (116 - Repair_Point_Ysite) + RowAttribute[116].RightBorder;
            RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
        }
    }
    // flag 9-10: 已出环半宽处理
    if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10) {
        for (int Ysite = 119; Ysite > ImageParameter.OFFLine; Ysite--) {
            RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}

// ========================================== 右圆环处理 ==========================================
void Element_Handle_Right_Rings() {
    int num = 0;
    for (int Ysite = 110; Ysite > 48; Ysite--) {
        if (RowAttribute[Ysite].IsRightFind == 'W') num++;
        if (RowAttribute[Ysite + 6].IsRightFind == 'W' && RowAttribute[Ysite + 4].IsRightFind == 'W'
            && RowAttribute[Ysite + 2].IsRightFind == 'W' && RowAttribute[Ysite].IsRightFind == 'T')
            break;
    }
    if (ImageParameter.image_element_rings_flag == 1 && num > 40) {
        ImageParameter.image_element_rings_flag = 2;
    }
    if (ImageParameter.image_element_rings_flag == 2 && num < 40) {  // 原 20 → 40
        ImageParameter.image_element_rings_flag = 5;
    }
    if (ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_L > 32) {
        ImageParameter.image_element_rings_flag = 6;
    }
    if (ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_L < 10) {
        ImageParameter.image_element_rings_flag = 7;
    }
    // flag7→8: 环内 → 出环
    if (ImageParameter.image_element_rings_flag == 7) {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 90; Ysite > ImageParameter.OFFLine + 14; Ysite--) {  // 原 45 → 90
            if (RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 4].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 4].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 2].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 2].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 8].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 8].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 10].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 10].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 12].LeftBorder
                && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 12].LeftBorder) {
                Point_Xsite = RowAttribute[Ysite].LeftBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 44) {
            ImageParameter.image_element_rings_flag = 8;
        }
    }
    if (ImageParameter.image_element_rings_flag == 8) {
        if (ImageParameter.WhiteLine_R < 24        // 原 12 → 24
            && ImageParameter.OFFLine < 36) {      // 原 18 → 36
            ImageParameter.image_element_rings_flag = 9;
        }
    }
    if (ImageParameter.image_element_rings_flag == 9) {
        int num = 0;
        for (int Ysite = 90; Ysite > 20; Ysite--) {  // 原 45→10 → 90→20
            if (RowAttribute[Ysite].IsRightFind == 'W') num++;
        }
        if (num < 16) {
            ImageParameter.image_element_rings_flag = 0;
            ImageParameter.image_element_rings = 0;
            ImageParameter.ring_big_small = 0;
        }
    }
    printf("R_flag:%d num:%d Det=%d\n", ImageParameter.image_element_rings_flag, num, ImageParameter.Det_True);

    /*************************************** 右环岛处理 **************************************/
    if (ImageParameter.image_element_rings_flag >= 1
        && ImageParameter.image_element_rings_flag <= 4) {
        for (int Ysite = 119; Ysite > ImageParameter.OFFLine; Ysite--) {
            RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
    // flag 5-6: 进环补线
    if (ImageParameter.image_element_rings_flag == 5 || ImageParameter.image_element_rings_flag == 6) {
        int flag_Xsite_1 = 0;
        int flag_Ysite_1 = 0;
        float Slope_Right_Rings = 0;
        for (Ysite = 100; Ysite > ImageParameter.OFFLine; Ysite--) {
            for (Xsite = RowAttribute[Ysite].LeftBorder + 1; Xsite < RowAttribute[Ysite].RightBorder - 1; Xsite++) {
                if (heheImage[Ysite][Xsite] == 1 && heheImage[Ysite][Xsite + 1] == 0) {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = Xsite;
                    break;
                }
            }
            if (flag_Ysite_1 != 0) break;
        }
        if (flag_Ysite_1 == 0) {
            for (Ysite = ImageParameter.OFFLine + 1; Ysite < 60; Ysite++) {
                if (RowAttribute[Ysite].IsRightFind == 'T' && RowAttribute[Ysite + 1].IsRightFind == 'T'
                    && RowAttribute[Ysite + 2].IsRightFind == 'W'
                    && abs(RowAttribute[Ysite].RightBorder - RowAttribute[Ysite + 2].RightBorder) > 20) {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = RowAttribute[flag_Ysite_1].RightBorder;
                    ImageParameter.OFFLine = (uint8_t)Ysite;
                    break;
                }
            }
        }
        int bottom_x = 144;  // 原 72 → 144
        for (int x = 0; x < 144; x++) {
            if (heheImage[110][x] == 0 && heheImage[110][x + 1] == 1) {  // 原 55 → 110
                bottom_x = x;
                break;
            }
        }
        if (flag_Ysite_1 != 0 && bottom_x < 158) {
            Slope_Right_Rings = (float)(bottom_x - flag_Xsite_1) / (float)(119 - flag_Ysite_1);
        }
        if (flag_Ysite_1 != 0) {
            for (Ysite = flag_Ysite_1; Ysite < IPSH; Ysite++) {
                RowAttribute[Ysite].LeftBorder = flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
                RowAttribute[Ysite].Center = (RowAttribute[Ysite].LeftBorder + RowAttribute[Ysite].RightBorder) / 2;
            }
            RowAttribute[flag_Ysite_1].LeftBorder = flag_Xsite_1;
            for (Ysite = flag_Ysite_1 - 1; Ysite > 20; Ysite--) {
                for (Xsite = RowAttribute[Ysite + 1].LeftBorder + 16; Xsite > RowAttribute[Ysite + 1].LeftBorder - 8; Xsite--) {
                    if (heheImage[Ysite][Xsite] == 1 && heheImage[Ysite][Xsite - 1] == 0) {
                        RowAttribute[Ysite].LeftBorder = Xsite;
                        RowAttribute[Ysite].Wide = RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
                        RowAttribute[Ysite].Center = (RowAttribute[Ysite].LeftBorder + RowAttribute[Ysite].RightBorder) / 2;
                        break;
                    }
                }
                if (RowAttribute[Ysite].Wide > 16 && RowAttribute[Ysite].LeftBorder > RowAttribute[Ysite + 4].LeftBorder) {
                    continue;
                }
                else {
                    ImageParameter.OFFLine = Ysite + 2;
                    break;
                }
            }
        }
    }
    // flag 8: 大圆环出环补线
    if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1) {
        Repair_Point_Xsite = 80;  // 原 40 → 80
        Repair_Point_Ysite = 0;
        for (int Ysite = 100; Ysite > 10; Ysite--) {
            if (heheImage[Ysite][80] == 1 && heheImage[Ysite - 1][80] == 0) {
                Repair_Point_Xsite = 80;
                Repair_Point_Ysite = Ysite - 1;
                ImageParameter.OFFLine = Ysite + 1;
                break;
            }
        }
        for (int Ysite = 114; Ysite > Repair_Point_Ysite - 6; Ysite--) {
            RowAttribute[Ysite].LeftBorder = (RowAttribute[116].LeftBorder - Repair_Point_Xsite) * (Ysite - 116) / (116 - Repair_Point_Ysite) + RowAttribute[116].LeftBorder;
            RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
        }
    }
    // flag 8: 小圆环出环补线
    if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2) {
        Repair_Point_Xsite = 100;  // 原 50 → 100 (曾为 65→130)
        Repair_Point_Ysite = 0;
        for (int Ysite = 80; Ysite > 10; Ysite--) {
            if (heheImage[Ysite][116] == 1 && heheImage[Ysite - 1][116] == 0) {  // 原 58 → 116
                Repair_Point_Xsite = 100;
                Repair_Point_Ysite = Ysite - 1;
                ImageParameter.OFFLine = Ysite + 1;
                break;
            }
        }
        for (int Ysite = 110; Ysite > Repair_Point_Ysite - 6; Ysite--) {
            RowAttribute[Ysite].LeftBorder = (RowAttribute[116].LeftBorder - Repair_Point_Xsite) * (Ysite - 116) / (116 - Repair_Point_Ysite) + RowAttribute[116].LeftBorder;
            RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
        }
    }
    // flag 9-10: 已出环半宽处理
    if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10) {
        for (int Ysite = 119; Ysite > ImageParameter.OFFLine; Ysite--) {
            RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

// ========================================== 斑马线判断 (圈数计数+入库) ==========================================
void Element_Judgment_Zebra() {
    const int TARGET_LAPS = 2;
    const uint32_t LOCK_MS = 8000;

    static uint8_t inited = 0;
    static int lap_count = 0;
    static uint32_t last_lap_time = 0;
    static uint8_t zebra_pending = 0;
    static uint32_t zebra_time = 0;

    extern bool car_started;

    if (!car_started) return;
    if (!inited) { last_lap_time = Get_Time_Ms(); inited = 1; }

    if (zebra_pending) {
        if (Get_Time_Ms() - zebra_time >= 1000) {
            SystemData.Stop = 1;  // 入库停车
            zebra_pending = 0;
        }
        return;
    }
    if (SystemData.Stop) return;

    if (Get_Time_Ms() - last_lap_time < LOCK_MS) return;

    int NUM = 0, net = 0;
    if (ImageParameter.OFFLineBoundary < 40) {  // 原 20 → 40
        for (int Ysite = 80; Ysite < 106; Ysite++) {  // 原 40→53 → 80→106
            net = 0;
            for (int Xsite = RowAttribute[Ysite].LeftBoundary; Xsite < RowAttribute[Ysite].RightBoundary; Xsite++) {
                if (heheImage[Ysite][Xsite] == 0 && heheImage[Ysite][Xsite + 1] == 1) {
                    net++;
                    if (net > 2) NUM++;
                }
            }
        }
    }

    if (NUM >= 5) {
        lap_count++;
        last_lap_time = Get_Time_Ms();
        cout << "lap: " << lap_count << endl;

        if (lap_count >= TARGET_LAPS) {
            zebra_pending = 1;
            zebra_time = Get_Time_Ms();
        }
    }
}

// ========================================== 元素检测总入口 ==========================================
void Element_Test(void) {
    detect_red_color_block();
    Straightacc_Test();

    // 圆环检测条件：未在圆环状态 + 丢线不严重
    if (ImageParameter.image_element_rings_flag == 0
        && ImageParameter.WhiteLine < 10) {  // 原 5 → 10
        Element_Judgment_Left_Rings();
        Element_Judgment_Right_Rings();
    }
    Element_Judgment_Zebra();
}

// ========================================== 元素处理总入口 ==========================================
void Element_Handle(void) {
    element_I();  // 十字处理

    if (ImageParameter.image_element_rings == 1)
        Element_Handle_Left_Rings();
    else if (ImageParameter.image_element_rings == 2)
        Element_Handle_Right_Rings();

    RouteFilter();
    Emergency_Breaking();
}

// ========================================== 延长线绘制 ==========================================
void DrawExtensionLine(void) {
    if (ImageParameter.Road_type != Straight &&
        ImageParameter.Road_type != Cross_ture &&
        ImageParameter.Road_type != Ramp) {
        return;
    }

    // 左边界延长
    if (ExtenLFlag != 'F') {
        for (Ysite = 108; Ysite >= (ImageParameter.OFFLine + 4); Ysite--) {  // 原 54 → 108
            if (RowAttribute[Ysite].IsLeftFind == 'W') {
                if (Ysite + 1 < IPSH && RowAttribute[Ysite + 1].LeftBorder >= 140) {  // 原 70 → 140
                    ImageParameter.OFFLine = Ysite + 1;
                    break;
                }
                int search_y = Ysite;
                while (search_y >= (ImageParameter.OFFLine + 4)) {
                    search_y--;
                    if (search_y >= 2 &&
                        RowAttribute[search_y].IsLeftFind == 'T' &&
                        RowAttribute[search_y - 1].IsLeftFind == 'T' &&
                        RowAttribute[search_y - 2].IsLeftFind == 'T' &&
                        RowAttribute[search_y - 2].LeftBorder > 0 &&
                        RowAttribute[search_y - 2].LeftBorder < 140) {
                        left_FTSite = search_y - 2;
                        break;
                    }
                }
                if (left_FTSite > ImageParameter.OFFLine) {
                    DetL = (float)(RowAttribute[left_FTSite].LeftBorder - RowAttribute[TFSite].LeftBorder) /
                          (float)(left_FTSite - TFSite);
                    for (ytemp = TFSite; ytemp >= left_FTSite; ytemp--) {
                        RowAttribute[ytemp].LeftBorder = (int)(DetL * (ytemp - TFSite)) + RowAttribute[TFSite].LeftBorder;
                    }
                }
            } else {
                TFSite = Ysite + 2;
            }
        }
    }

    // 右边界延长
    TFSite = 0;
    if (ExtenRFlag != 'F') {
        for (Ysite = 108; Ysite >= (ImageParameter.OFFLine + 4); Ysite--) {
            if (RowAttribute[Ysite].IsRightFind == 'W') {
                if (Ysite + 1 < IPSH && RowAttribute[Ysite + 1].RightBorder <= 20) {  // 原 10 → 20
                    ImageParameter.OFFLine = Ysite + 1;
                    break;
                }
                int search_y = Ysite;
                while (search_y >= (ImageParameter.OFFLine + 4)) {
                    search_y--;
                    if (search_y >= 2 &&
                        RowAttribute[search_y].IsRightFind == 'T' &&
                        RowAttribute[search_y - 1].IsRightFind == 'T' &&
                        RowAttribute[search_y - 2].IsRightFind == 'T' &&
                        RowAttribute[search_y - 2].RightBorder < 140 &&
                        RowAttribute[search_y - 2].RightBorder > 20) {
                        right_FTSite = search_y - 2;
                        break;
                    }
                }
                if (right_FTSite > ImageParameter.OFFLine) {
                    DetR = (float)(RowAttribute[right_FTSite].RightBorder - RowAttribute[TFSite].RightBorder) /
                          (float)(right_FTSite - TFSite);
                    for (ytemp = TFSite; ytemp >= right_FTSite; ytemp--) {
                        RowAttribute[ytemp].RightBorder = (int)(DetR * (ytemp - TFSite)) + RowAttribute[TFSite].RightBorder;
                    }
                }
            } else {
                TFSite = Ysite + 2;
            }
        }
    }

    // 重新计算中线
    for (Ysite = 119; Ysite >= ImageParameter.OFFLine; Ysite--) {
        RowAttribute[Ysite].Center = (RowAttribute[Ysite].LeftBorder + RowAttribute[Ysite].RightBorder) / 2;
        RowAttribute[Ysite].Wide = RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
    }
}

// ========================================== 前瞻误差计算 ==========================================
int Det_True = 0;

void Prospective_error(void) {
    float DetTemp = 0;
    int TowPoint = 0;
    float UnitAll = 0;
    int Ysite;

    TowPoint = ImageParameter.OFFLine + ImageParameter.TowPoint;

    if (TowPoint < ImageParameter.OFFLine)
        TowPoint = ImageParameter.OFFLine + 1;
    if (TowPoint >= 98) TowPoint = 98;  // 原 49 → 98

    if ((TowPoint - 5) >= ImageParameter.OFFLine) {
        for (Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
            DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (RowAttribute[Ysite].Center);
            UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[TowPoint].Center + DetTemp) / (UnitAll + 1);
    }
    else if (TowPoint > ImageParameter.OFFLine) {
        for (Ysite = ImageParameter.OFFLine; Ysite < TowPoint; Ysite++) {
            DetTemp += Weighting[TowPoint - Ysite - 1] * (RowAttribute[Ysite].Center);
            UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + TowPoint - ImageParameter.OFFLine); Ysite > TowPoint; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[Ysite].Center + DetTemp) / (UnitAll + 1);
    }
    else if (ImageParameter.OFFLine < 98) {
        for (Ysite = (ImageParameter.OFFLine + 3); Ysite > ImageParameter.OFFLine; Ysite--) {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[ImageParameter.OFFLine].Center + DetTemp) / (UnitAll + 1);
    }
    else
        DetTemp = Det_True;

    Det_True = (int)DetTemp;
    ImageParameter.Det_True = Det_True;
}

// ========================================== 出界保护 ==========================================
void Emergency_Breaking(void) {
    extern bool car_started;
    if (car_started && !SystemData.Stop) {
        int black_count = 0;
        int total_pixels = 0;
        // 检查底部10行 (原 41→46 → 82→92)
        for (int y = 82; y < 92; y++) {
            for (int x = 0; x < IPSW; x++) {
                if (heheImage[y][x] == 0) black_count++;
                total_pixels++;
            }
        }
        if (total_pixels > 0 && (black_count * 100 / total_pixels) > 80) {
            SystemData.Stop = 1;
            printf("Emergency: off-track detected!\n");
        }
    }
}

// ========================================== 打印边界调试信息 ==========================================
void PrintBorders(const RowAttributetypedef* rowAttributes, int numRows) {
    for (int y = 0; y < numRows; y++) {
        int left = rowAttributes[y].LeftBorder;
        int right = rowAttributes[y].RightBorder;
        cout << "Row " << y << ": Left=" << left << ", Right=" << right << endl;
    }
}

// ========================================== 初始化 NCNN 模型 (已禁用) ==========================================
static void init_ncnn_model(void) {
    ncnn_initialized = 0;
}

// ========================================== 红色物块识别 ==========================================
int detect_red_color_block(void) {
    init_ncnn_model();

    red_block_detected = 0;
    red_block_center_x = -1;
    red_block_center_y = -1;
    extract_region_x = -1;
    extract_region_y = -1;

    // 创建红色掩码
    Mat red_mask(IPSH, IPSW, CV_8UC1);
    for (int i = 0; i < IPSH; i++) {
        for (int j = 0; j < IPSW; j++) {
            uint8_t r = color_image[i][j][2];
            uint8_t g = color_image[i][j][1];
            uint8_t b = color_image[i][j][0];
            red_mask.at<uchar>(i, j) = (r > 100 && r > g + 30 && r > b + 10) ? 255 : 0;
        }
    }

    Mat labels, stats, centroids;
    int numLabels = connectedComponentsWithStats(red_mask, labels, stats, centroids, 8, CV_32S);

    if (numLabels <= 1) {
        consecutive_detect_count = 0;
        return 0;
    }

    struct BlockInfo {
        int center_x, center_y;
        int area;
    };
    std::vector<BlockInfo> validBlocks;

    int minArea = 30;
    for (int i = 1; i < numLabels; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        if (area < minArea) continue;

        int left = stats.at<int>(i, CC_STAT_LEFT);
        int top = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);

        int center_x = left + width / 2;
        int center_y = top + height / 2;

        int rowIndex = center_y;
        if (rowIndex < 0) rowIndex = 0;
        if (rowIndex >= IPSH) rowIndex = IPSH - 1;

        int leftBorder = RowAttribute[rowIndex].LeftBorder;
        int rightBorder = RowAttribute[rowIndex].RightBorder;

        // 色块中心必须在赛道边界之间 (放宽容差)
        if (center_x <= leftBorder - 40 || center_x >= rightBorder + 40) continue;

        validBlocks.push_back({center_x, center_y, area});
    }

    if (validBlocks.empty()) {
        consecutive_detect_count = 0;
        return 0;
    }

    // 从下往上找，取最下方的有效色块
    BlockInfo targetBlock = validBlocks[0];
    for (auto& block : validBlocks) {
        if (block.center_y > targetBlock.center_y) {
            targetBlock = block;
        }
    }

    consecutive_detect_count++;
    if (consecutive_detect_count >= 6) {
        red_block_center_x = targetBlock.center_x;
        red_block_center_y = targetBlock.center_y;
        red_block_detected = 1;
    }
    printf("[RED_BLOCK] DETECTED! center=(%d, %d) area=%d validBlocks=%d\n",
       red_block_center_x, red_block_center_y, targetBlock.area, (int)validBlocks.size());

    extract_region_size = 64;
    int src_size = 32;
    int extract_y = red_block_center_y - 21;
    int extract_x = red_block_center_x - 16;

    if (extract_y < 0) extract_y = 0;
    if (extract_y > IPSH - src_size) extract_y = IPSH - src_size;
    if (extract_x < 0) extract_x = 0;
    if (extract_x > IPSW - src_size) extract_x = IPSW - src_size;

    extract_region_x = extract_x;
    extract_region_y = extract_y;

    detect_red_block_classify();
    return 1;
}

// ========================================== 红色物块分类 (已禁用) ==========================================
int detect_red_block_classify(void) {
    return 0;
}

// ========================================== 图像处理主函数 ==========================================
void ImageProcess(void) {
    // flag 保护性检查
    if (ImageParameter.image_element_rings_flag < 0 ||
        ImageParameter.image_element_rings_flag > 9) {
        ImageParameter.image_element_rings_flag = 0;
        ImageParameter.image_element_rings = 0;
        ImageParameter.ring_big_small = 0;
    }

    ImageParameter.OFFLine = 10;   // 原 2 → 10
    ImageParameter.WhiteLine = 0;
    ImageParameter.WhiteLine_L = 0;
    ImageParameter.WhiteLine_R = 0;
    for (Ysite = 119; Ysite >= ImageParameter.OFFLine; Ysite--) {
        RowAttribute[Ysite].IsLeftFind = 'F';
        RowAttribute[Ysite].IsRightFind = 'F';
        RowAttribute[Ysite].LeftBorder = 0;
        RowAttribute[Ysite].RightBorder = 159;  // 原 79 → 159
    }

    Thershold_separation_Otsu();
    Bin_Image_Filter();
    DrawLinesBasic();
    DrawLinesProcess();
    DrawExtensionLine();

    // 八邻域巡线作为元素判断依据
    Search_Border_OTSU(heheImage, IPSH, IPSW, IPSH - 1);

    // 元素识别
    Element_Test();

    // 元素处理
    Element_Handle();

    // 更新前瞻
    Update_Dynamic_Lookahead();

    // 前瞻误差计算
    Prospective_error();

    // 红色物块避让
    if (red_block_detected && ImageParameter.image_element_rings_flag == 0) {
        const int AVOID_OFFSET = 80;  // 原 40 → 80 (适应翻倍后的图像宽度)

        int leftSpace = red_block_center_x - RowAttribute[80].LeftBorder;  // 原 40 → 80
        int rightSpace = RowAttribute[80].RightBorder - red_block_center_x;

        if (leftSpace > rightSpace) {
            ImageParameter.Det_True -= AVOID_OFFSET;
        } else {
            ImageParameter.Det_True += AVOID_OFFSET;
        }
    }

    // 出界保护
    Emergency_Breaking();
}

// ========================================== Zouma 兼容接口: image_proc() ==========================================
void image_proc() {
    // 1. 获取图像帧并 resize 到 160x120
    cv::Mat frame_resized;
    cv::resize(uvc.frame_mjpg, frame_resized, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);

    // 2. 提取灰度图 → Cramp_image[120][160]
    cv::Mat frame_gray_small;
    cv::cvtColor(frame_resized, frame_gray_small, cv::COLOR_BGR2GRAY);
    img_gray = reinterpret_cast<uint8_t*>(frame_gray_small.ptr(0));

    for (int i = 0; i < IPSH; i++) {
        for (int j = 0; j < IPSW; j++) {
            Cramp_image[i][j] = img_gray[i * IPSW + j];
        }
    }

    // 3. 提取彩色图像 (用于红色物块识别)
    {
        cv::Mat color_small;
        cv::resize(frame_resized, color_small, cv::Size(160, 120), 0, 0, cv::INTER_NEAREST);
        for (int i = 0; i < IPSH; i++) {
            for (int j = 0; j < IPSW; j++) {
                cv::Vec3b pixel = color_small.at<cv::Vec3b>(i, j);
                color_image[i][j][0] = pixel[0];  // B
                color_image[i][j][1] = pixel[1];  // G
                color_image[i][j][2] = pixel[2];  // R
            }
        }
    }

    // 4. 生成二值化图像供显示线程
    for (int i = 0; i < IPSW * IPSH; i++) {
        bin_img_data[i] = (img_gray[i] > start_thre) ? 255 : 0;
    }

    // 5. 执行 716 视觉管线
    ImageProcess();

    // 6. Det_True (像素列偏移) → onto (角度, 度)
    int pixel_offset = ImageParameter.Det_True - PictureCentring;  // -80 ~ +80
    float focal_length = 200.0f;  // 等效焦距 (需标定)
    onto = atan2f((float)pixel_offset, focal_length) * 180.0f / M_PI;

    // 限幅到 [-30, 30] 度
    if (onto > 30.0f) onto = 30.0f;
    if (onto < -30.0f) onto = -30.0f;
}

// ========================================== 空实现桩函数 ==========================================
void Cramping(void) {}
void Element_Handle_Small_Rings(void) {}
void Element_Handle_Big_Rings(void) {}
void Element_Judgment_Ramp(void) {}
void Check_Ring_State_Timeout(void) {}
