#ifndef SOCKET_H
#define SOCKET_H

// 平台相关头文件与库链接
#ifdef _WIN32
#   include <winsock2.h>
#   include <ws2tcpip.h>
#   pragma comment(lib, "ws2_32.lib")  // Windows下链接ws2_32库
typedef SOCKET SOCKET_HANDLE;  // Windows用SOCKET作为句柄类型
#else
#   include <sys/socket.h>
#   include <netinet/in.h>
#   include <arpa/inet.h>
#   include <unistd.h>
#   include <fcntl.h>
typedef int  SOCKET_HANDLE;    // Linux用int作为句柄类型
#   define INVALID_SOCKET  (-1)  // 跨平台无效句柄定义
#   define SOCKET_ERROR    (-1)
#endif

#include <stdbool.h>
#include <stdio.h>

// 全局对象声明（在源文件中定义，确保全局唯一）
#ifdef _WIN32
extern WSADATA wsaData;       // Windows下WSA初始化数据
extern bool    wsaInitialized; // 标记WSA是否已初始化
#endif

// 函数声明（仅声明，实现放在源文件中）
void InitializeSocketAPI(void);                   // 跨平台初始化Socket API
bool OpenSocket(SOCKET_HANDLE* sock, const char* ip, int port);  // 建立连接
void CloseSocket(SOCKET_HANDLE sock);             // 关闭套接字

#endif /* SOCKET_H */