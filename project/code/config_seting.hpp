#ifndef _CONFIG_SETTING_HPP_
#define _CONFIG_SETTING_HPP_
#include "my_global.hpp"
// ==================== 控制模式标志 ====================
extern uint8_t control_model;  // 0: PID模式, 1: LADRC模式

// ==================== LADRC 右轮参数 ====================
extern float lardc_r_h;
extern float lardc_r_r;
extern float lardc_r_wc;
extern float lardc_r_w0;
extern float lardc_r_b0;
extern float lardc_r_pwm_min;
extern float lardc_r_pwm_max;

// ==================== LADRC 左轮参数 ====================
extern float lardc_l_h;
extern float lardc_l_r;
extern float lardc_l_wc;
extern float lardc_l_w0;
extern float lardc_l_b0;
extern float lardc_l_pwm_min;
extern float lardc_l_pwm_max;

// ==================== PID 右轮参数 ====================
extern float pid_r_kp;
extern float pid_r_ts;
extern float pid_r_ki;
extern float pid_r_kd;
extern float pid_r_error_filter;
extern float pid_r_output_max;
extern float pid_r_output_min;
extern float pid_r_integral_max;
extern float pid_r_integral_min;

// ==================== PID 左轮参数 ====================
extern float pid_l_kp;
extern float pid_l_ts;
extern float pid_l_ki;
extern float pid_l_kd;
extern float pid_l_error_filter;
extern float pid_l_output_max;
extern float pid_l_output_min;
extern float pid_l_integral_max;
extern float pid_l_integral_min;

// ==================== 方向环 PD 参数 ====================
extern float onto_kp;
extern float onto_kp2;
extern float onto_kd;
extern float onto_limit;

// TCP 远程调参已废弃
// 接收缓存区
// extern std::string received;

// TCP 远程调参已废弃
// class TCPClient { ... };

void param_loading_from_file(const char* config_file);
void param_setting();
bool write_param_into_file();
void param_print();
// TCP 远程调参已废弃
// bool parseAndUpdateParameter(const std::string& line);
// void handleReadCommand();
// bool handleWriteCommand(TCPClient& client);

#endif