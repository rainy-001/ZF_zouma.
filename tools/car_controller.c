1/*********************************************************************************************************************
* 文件名称          car_controller.c
* 功能说明          PC 端 WASD 键盘遥控小车程序（Linux C 语言）
* 编译方式          gcc -o car_controller car_controller.c -Wall -O2
* 使用方法          ./car_controller <小车IP> [端口号，默认9090]
* 按键说明          W前进 S后退 A左转 D右转  Q加速 E减速  空格停车  Ctrl+C退出
* 适用平台          Linux x86_64
********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DEFAULT_PORT    9090
#define SEND_INTERVAL_MS 30    // 发送间隔(ms)，连按时持续发送

static int sock_fd = -1;
static struct termios old_termios;
static int running = 1;

/* ====== 终端设置 ====== */
static void term_enable_raw()
{
    tcgetattr(STDIN_FILENO, &old_termios);
    struct termios raw = old_termios;
    raw.c_lflag &= ~(ICANON | ECHO);      // 关闭规范模式和回显
    raw.c_iflag &= ~(IXON | ICRNL);       // 关闭 XON/XOFF 和 CR→NL
    raw.c_cc[VMIN] = 0;                   // 非阻塞读取
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void term_restore()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
}

/* ====== 信号处理 ====== */
static void sig_handler(int sig)
{
    (void)sig;
    running = 0;
}

/* ====== 打印帮助 ====== */
static void print_help()
{
    printf("\n");
    printf("========================================\n");
    printf("    WASD 小车遥控器\n");
    printf("========================================\n");
    printf("  W / ↑  前进\n");
    printf("  S / ↓  后退\n");
    printf("  A / ←  左转\n");
    printf("  D / →  右转\n");
    printf("  Q      加速 (+5)\n");
    printf("  E      减速 (-5)\n");
    printf("  空格   停车\n");
    printf("  X      急停\n");
    printf("  H      显示帮助\n");
    printf("  Ctrl+C 退出程序\n");
    printf("========================================\n");
    printf("\n");
}

/* ====== 发送指令 ====== */
static int send_cmd(const char *target_ip, int port, char cmd)
{
    if (sock_fd < 0) {
        // 创建 socket
        sock_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd < 0) {
            perror("socket");
            return -1;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (inet_pton(AF_INET, target_ip, &addr.sin_addr) <= 0) {
            fprintf(stderr, "无效的IP地址: %s\n", target_ip);
            close(sock_fd);
            sock_fd = -1;
            return -1;
        }

        if (connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("connect");
            close(sock_fd);
            sock_fd = -1;
            return -1;
        }

        printf("[连接成功] 已连接到 %s:%d\n", target_ip, port);
    }

    ssize_t n = send(sock_fd, &cmd, 1, 0);
    if (n <= 0) {
        perror("send");
        close(sock_fd);
        sock_fd = -1;
        return -1;
    }

    return 0;
}

/* ====== 按键名称 ====== */
static const char* key_name(char c)
{
    switch (c) {
        case 'w': return "前进";
        case 's': return "后退";
        case 'a': return "左转";
        case 'd': return "右转";
        case 'q': return "加速";
        case 'e': return "减速";
        case ' ': return "停车";
        case 'x': return "急停";
        default:  return "?";
    }
}

/* ====== 主函数 ====== */
int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s <小车IP地址> [端口号，默认%d]\n", argv[0], DEFAULT_PORT);
        fprintf(stderr, "示例: %s 192.168.79.125 %d\n", argv[0], DEFAULT_PORT);
        return 1;
    }

    const char *target_ip = argv[1];
    int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORT;

    // 注册信号处理
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    // 设置终端为 raw 模式
    term_enable_raw();
    print_help();

    // 有效按键集合
    const char valid_keys[] = "wasd qex";
    char last_key = 0;

    printf("等待按键... (按 H 显示帮助)\n");

    while (running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = SEND_INTERVAL_MS * 1000;  // 30ms

        int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
            char buf[16];
            ssize_t n = read(STDIN_FILENO, buf, sizeof(buf) - 1);
            if (n <= 0) break;

            // 检查 Ctrl+C
            if (buf[0] == 3) {
                running = 0;
                break;
            }

            char key = buf[0];

            // 帮助键
            if (key == 'h' || key == 'H') {
                print_help();
                last_key = 0;
                continue;
            }

            // 检查是否为有效按键
            int valid = 0;
            for (int i = 0; valid_keys[i]; i++) {
                if (key == valid_keys[i]) {
                    valid = 1;
                    break;
                }
            }

            if (!valid) {
                // 忽略无效按键
                last_key = 0;
                continue;
            }

            // 发送指令
            if (send_cmd(target_ip, port, key) == 0) {
                if (key != last_key) {
                    printf(">>> %s ('%c')\n", key_name(key), key);
                }
                last_key = key;
            } else {
                // 连接断开，尝试重连
                printf("[连接断开，等待重连...]\n");
                last_key = 0;
                sleep(1);
            }
        } else if (last_key != 0) {
            // select 超时但上次按键还在 — 持续发送（处理长按）
            if (send_cmd(target_ip, port, last_key) != 0) {
                printf("[连接断开，等待重连...]\n");
                last_key = 0;
                sleep(1);
            }
        } else {
            last_key = 0;
        }
    }

    // 退出前发送急停
    printf("\n正在退出，发送急停指令...\n");
    if (sock_fd >= 0) {
        char stop = ' ';
        send(sock_fd, &stop, 1, 0);
        close(sock_fd);
        sock_fd = -1;
    }

    // 恢复终端设置
    term_restore();
    printf("程序已退出\n");

    return 0;
}