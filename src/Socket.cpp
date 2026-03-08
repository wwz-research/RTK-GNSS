#include "Socket.h" 

// 全局对象定义（仅在源文件中定义一次，避免多重定义）
#ifdef _WIN32
WSADATA wsaData;       // 定义Windows下的WSA数据
bool    wsaInitialized = false;  // 初始化标记，默认未初始化
#endif

/* 跨平台初始化Socket API */
void InitializeSocketAPI(void)
{
#ifdef _WIN32
    if (!wsaInitialized)  // 仅初始化一次
    {
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        {
            perror("WSAStartup failed");  // 初始化失败提示
        }
        wsaInitialized = true;  // 标记为已初始化
    }
#endif  // Linux不需要额外初始化，直接使用系统Socket API
}

/* 建立TCP连接 */
bool OpenSocket(SOCKET_HANDLE* sock, const char* ip, int port)
{
    struct sockaddr_in serverAddr;  // 服务器地址结构

    InitializeSocketAPI();  // 确保Socket API已初始化

    // 创建套接字（跨平台处理）
#ifdef _WIN32
    * sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);  // 避免与typedef冲突，用::调用全局函数
#else
    * sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#endif

    if (*sock == INVALID_SOCKET)
    {
        perror("Socket creation failed");  // 创建失败提示
        return false;
    }

    // 初始化服务器地址
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(static_cast<short>(port));  // 端口转换为网络字节序
    if (inet_pton(AF_INET, ip, &serverAddr.sin_addr) <= 0)  // IP地址转换为网络字节序
    {
        perror("Invalid IP address");  // IP无效提示
        return false;
    }

    // 连接服务器
    if (connect(*sock, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR)
    {
#ifdef _WIN32
        printf("连接失败，错误代码：%d\n", WSAGetLastError());  // Windows下错误码
        closesocket(*sock);
#else
        perror("Connection failed");  // Linux下错误提示
        close(*sock);
#endif
        * sock = INVALID_SOCKET;  // 标记为无效句柄
        return false;
    }

    return true;  // 连接成功
}

/* 关闭套接字并清理资源 */
void CloseSocket(SOCKET_HANDLE sock)
{
    if (sock == INVALID_SOCKET)
        return;  // 无效句柄直接返回

    // 关闭套接字（跨平台处理）
#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif

    // Windows下清理WSA资源（仅在初始化后清理）
#ifdef _WIN32
    if (wsaInitialized)
    {
        WSACleanup();
        wsaInitialized = false;  // 重置初始化标记
    }
#endif
}