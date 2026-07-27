/*********************************************************************************************************************
* 文件名称          remote_control.cpp
* 功能说明          WASD 键盘远程遥控小车 - TCP 服务端实现
* 适用平台          LS2K0300
********************************************************************************************************************/

#include "remote_control.hpp"

RemoteControl::RemoteControl()
    : server_fd(-1)
    , client_fd(-1)
    , client_connected(false)
    , last_cmd(0)
{
}

RemoteControl::~RemoteControl()
{
    if (client_fd >= 0) {
        close(client_fd);
        client_fd = -1;
    }
    if (server_fd >= 0) {
        close(server_fd);
        server_fd = -1;
    }
    client_connected = false;
}

bool RemoteControl::init_server()
{
    // 创建 socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("RemoteControl: socket create failed");
        return false;
    }

    // 设置 SO_REUSEADDR，快速重启
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 设置非阻塞
    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    // 绑定
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(REMOTE_PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("RemoteControl: bind failed");
        close(server_fd);
        server_fd = -1;
        return false;
    }

    // 监听
    if (listen(server_fd, 1) < 0) {
        perror("RemoteControl: listen failed");
        close(server_fd);
        server_fd = -1;
        return false;
    }

    printf("[RemoteControl] TCP server listening on port %d\n", REMOTE_PORT);
    return true;
}

bool RemoteControl::receive_command(char &cmd)
{
    // 1. 如果还没初始化，先初始化
    if (server_fd < 0) {
        if (!init_server()) {
            return false;
        }
    }

    // 2. 如果还没有客户端连接，尝试 accept
    if (!client_connected) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_fd >= 0) {
            // 设置客户端 socket 为非阻塞
            int flags = fcntl(client_fd, F_GETFL, 0);
            fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
            client_connected = true;
            printf("[RemoteControl] Client connected: %s\n", inet_ntoa(client_addr.sin_addr));
        }
        // accept 失败（无连接请求）属于正常情况，返回 false 即可
    }

    // 3. 尝试接收数据
    if (client_connected) {
        char buf[16];
        ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        
        if (n > 0) {
            // 收到数据，取最后一个字符作为指令
            cmd = buf[n - 1];
            last_cmd = cmd;
            return true;
        } else if (n == 0) {
            // 客户端正常断开
            printf("[RemoteControl] Client disconnected\n");
            close_client();
        } else {
            // n < 0: 错误或暂无数据
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // 真正的错误，断开连接
                perror("RemoteControl: recv error");
                close_client();
            }
        }
    }

    return false;
}

void RemoteControl::close_client()
{
    if (client_fd >= 0) {
        close(client_fd);
        client_fd = -1;
    }
    client_connected = false;
    last_cmd = 0;
}