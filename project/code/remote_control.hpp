/*********************************************************************************************************************
* 文件名称          remote_control.hpp
* 功能说明          WASD 键盘远程遥控小车 - TCP 服务端接收单字符指令
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-07-25        Cline                       first version
********************************************************************************************************************/

#ifndef __REMOTE_CONTROL_HPP__
#define __REMOTE_CONTROL_HPP__

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ================================================================================================================
 *                                           遥控参数配置
 * ================================================================================================================ */
#define REMOTE_PORT             9090            // TCP 监听端口
#define HEARTBEAT_TIMEOUT_MS    500             // 心跳超时(ms)，超时自动停车
#define STEER_AMOUNT            15.0f           // 转向修正量
#define MAX_CRUISING_SPEED      60.0f           // 最大巡航速度
#define MIN_CRUISING_SPEED      0.0f            // 最小巡航速度
#define SPEED_STEP              5.0f            // 速度调节步长

class RemoteControl {
private:
    int server_fd;                              // 服务端 socket
    int client_fd;                              // 客户端 socket
    bool client_connected;                      // 客户端连接标志
    char last_cmd;                              // 最后收到的指令
    bool init_server();                         // 初始化 TCP 服务端

    RemoteControl(const RemoteControl&) = delete;
    RemoteControl& operator=(const RemoteControl&) = delete;

public:
    RemoteControl();
    ~RemoteControl();

    // 非阻塞接收一个字符指令，返回 true 表示收到新指令
    bool receive_command(char &cmd);
    
    // 是否有客户端连接
    bool is_connected() const { return client_connected; }
    
    // 关闭当前客户端连接（保留 server_fd 继续监听）
    void close_client();
};

#endif // __REMOTE_CONTROL_HPP__